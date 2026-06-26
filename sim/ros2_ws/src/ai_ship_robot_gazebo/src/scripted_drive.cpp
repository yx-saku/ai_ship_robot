#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace ai_ship_robot_gazebo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinControlRemainingSec = 0.05;
constexpr double kDefaultPositionTolerance = 0.05;
constexpr double kDefaultYawTolerance = kPi / 180.0;

struct PlanarPose
{
  double x{};
  double y{};
  double yaw{};
};

struct AbsoluteMoveToPoseTarget
{
  double x{};
  double y{};
  double position_tolerance{};
  double yaw{};
  double yaw_tolerance{};
};

double normalize_angle(const double angle)
{
  auto normalized = std::fmod(angle + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

double yaw_from_odometry(const nav_msgs::msg::Odometry & odometry)
{
  const auto & orientation = odometry.pose.pose.orientation;
  const auto siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const auto cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class ScriptedDrive : public rclcpp::Node
{
public:
  ScriptedDrive()
  : Node("scripted_drive")
  {
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("scenario_file", "");
    declare_parameter<double>("start_delay_sec", 0.0);
    declare_parameter<double>("publish_rate", 10.0);
    declare_parameter<double>("max_linear_speed", 0.5);
    declare_parameter<double>("max_angular_speed", 1.0);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    scenario_file_ = get_parameter("scenario_file").as_string();
    start_delay_sec_ = get_parameter("start_delay_sec").as_double();
    publish_rate_ = get_parameter("publish_rate").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_pose_ = PlanarPose{message->pose.pose.position.x, message->pose.pose.position.y, yaw_from_odometry(*message)};
      });
    scenario_ = load_scenario_file(scenario_file_);
  }

  void run()
  {
    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be > 0.");
    }
    if (max_linear_speed_ <= 0.0) {
      throw std::invalid_argument("max_linear_speed must be > 0.");
    }
    if (max_angular_speed_ <= 0.0) {
      throw std::invalid_argument("max_angular_speed must be > 0.");
    }
    if (!wait_for_active_sim_time()) {
      return;
    }
    if (start_delay_sec_ > 0.0) {
      RCLCPP_INFO(get_logger(), "Waiting %.2f sec before scenario start (sim time)", start_delay_sec_);
      if (!wait_for_sim_duration(start_delay_sec_)) {
        return;
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    RCLCPP_INFO(get_logger(), "Starting scripted drive scenario");
    run_steps(period);
  }

  void stop()
  {
    publish_twist(0.0, 0.0, 0.0);
  }

private:
  std::optional<rclcpp::Time> wait_for_active_sim_time()
  {
    rclcpp::WallRate rate(100.0);

    // use_sim_timeの/clockを受信するまで待ち、0秒基準でdurationを計算して即終了する誤動作を防ぐ。
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      const auto current_time = get_clock()->now();
      if (current_time.nanoseconds() != 0) {
        return current_time;
      }
      rate.sleep();
    }
    return std::nullopt;
  }

  bool wait_for_sim_duration(const double duration_sec)
  {
    const auto active_time = wait_for_active_sim_time();
    if (!active_time) {
      return false;
    }
    auto start_time = *active_time;
    const auto target_duration = rclcpp::Duration::from_seconds(duration_sec);
    rclcpp::WallRate rate(100.0);

    // Gazeboの時刻進行に同期して待機し、real_time_factorの影響を正しく反映する。
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      const auto current_time = get_clock()->now();
      if (current_time.nanoseconds() == 0) {
        rate.sleep();
        continue;
      }
      if (current_time < start_time) {
        start_time = current_time;
        rate.sleep();
        continue;
      }
      if ((current_time - start_time) >= target_duration) {
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  void run_steps(const std::chrono::duration<double> period)
  {
    for (std::size_t index = 0; rclcpp::ok() && index < scenario_.steps.size(); ++index) {
      const auto & step = scenario_.steps[index];
      RCLCPP_INFO(get_logger(), "Step %zu: %s", index + 1, format_step_command(step).c_str());

      bool completed = false;
      if (step.type == ScenarioStepType::Velocity) {
        completed = run_velocity_step(step, period);
      } else if (step.type == ScenarioStepType::MoveToPose) {
        completed = run_move_to_pose_step(step, period);
      }

      if (!completed) {
        stop();
        throw std::runtime_error("Step " + std::to_string(index + 1) + " failed.");
      }
      stop();
      RCLCPP_INFO(get_logger(), "Step %zu: stop", index + 1);
    }
  }

  bool prepare_step_time(const ScenarioStep & step, rclcpp::Time & step_start_time, rclcpp::Time & deadline)
  {
    const auto active_step_time = wait_for_active_sim_time();
    if (!active_step_time) {
      return false;
    }
    step_start_time = *active_step_time;
    deadline = step_start_time + rclcpp::Duration::from_seconds(step.duration_sec);
    return true;
  }

  void adjust_time_if_reset(
    const ScenarioStep & step, const rclcpp::Time & current_time, rclcpp::Time & step_start_time,
    rclcpp::Time & deadline)
  {
    if (current_time < step_start_time) {
      step_start_time = current_time;
      deadline = current_time + rclcpp::Duration::from_seconds(step.duration_sec);
    }
  }

  bool run_velocity_step(const ScenarioStep & step, const std::chrono::duration<double> period)
  {
    rclcpp::Time step_start_time;
    rclcpp::Time deadline;
    if (!prepare_step_time(step, step_start_time, deadline)) {
      return false;
    }

    // 速度stepは従来通り一定Twistを出し、終了判定だけGazebo時刻で行う。
    rclcpp::WallRate rate(1.0 / period.count());
    while (rclcpp::ok()) {
      publish_twist(step.linear_x, step.linear_y, step.angular_z);
      rclcpp::spin_some(shared_from_this());
      const auto current_time = get_clock()->now();
      if (current_time.nanoseconds() == 0) {
        rate.sleep();
        continue;
      }
      adjust_time_if_reset(step, current_time, step_start_time, deadline);
      if (current_time >= deadline) {
        return true;
      }
      rate.sleep();
    }
    return false;
  }

  bool run_move_to_pose_step(const ScenarioStep & step, const std::chrono::duration<double> period)
  {
    rclcpp::Time step_start_time;
    rclcpp::Time deadline;
    if (!prepare_step_time(step, step_start_time, deadline)) {
      return false;
    }
    const auto target = resolve_move_to_pose_target(step);
    if (!target) {
      return false;
    }

    // duration_secを速度算出用の目標到達時間として使い、到達判定は許容誤差だけで行う。
    rclcpp::WallRate rate(1.0 / period.count());
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      const auto current_time = get_clock()->now();
      if (current_time.nanoseconds() == 0 || !latest_pose_) {
        rate.sleep();
        continue;
      }
      adjust_time_if_reset(step, current_time, step_start_time, deadline);
      const auto dx = target->x - latest_pose_->x;
      const auto dy = target->y - latest_pose_->y;
      const auto distance = std::hypot(dx, dy);
      const auto yaw_error = normalize_angle(target->yaw - latest_pose_->yaw);
      if (distance <= target->position_tolerance && std::abs(yaw_error) <= target->yaw_tolerance) {
        return true;
      }
      const auto remaining_sec = std::max((deadline - current_time).seconds(), kMinControlRemainingSec);
      const auto cos_yaw = std::cos(latest_pose_->yaw);
      const auto sin_yaw = std::sin(latest_pose_->yaw);
      auto linear_x = (cos_yaw * dx + sin_yaw * dy) / remaining_sec;
      auto linear_y = (-sin_yaw * dx + cos_yaw * dy) / remaining_sec;
      const auto angular_z = std::clamp(yaw_error / remaining_sec, -max_angular_speed_, max_angular_speed_);
      clamp_linear_velocity(linear_x, linear_y);
      publish_twist(linear_x, linear_y, angular_z);
      rate.sleep();
    }
    return false;
  }

  std::optional<AbsoluteMoveToPoseTarget> resolve_move_to_pose_target(const ScenarioStep & step)
  {
    if (!wait_for_odometry()) {
      return std::nullopt;
    }

    const auto start_pose = *latest_pose_;
    AbsoluteMoveToPoseTarget target;
    target.x = start_pose.x;
    target.y = start_pose.y;
    target.yaw = start_pose.yaw;
    target.position_tolerance = step.move_to_pose.has_position ? step.move_to_pose.position.tolerance : kDefaultPositionTolerance;
    target.yaw_tolerance = step.move_to_pose.has_yaw ? step.move_to_pose.yaw.tolerance : kDefaultYawTolerance;
    if (step.move_to_pose.mode == MoveToPoseMode::Absolute) {
      if (step.move_to_pose.has_position) {
        target.x = step.move_to_pose.position.x;
        target.y = step.move_to_pose.position.y;
      }
      if (step.move_to_pose.has_yaw) {
        target.yaw = step.move_to_pose.yaw.yaw;
      }
      return target;
    }

    // relはstep開始時のbase姿勢を基準にし、相対座標をodom座標の絶対目標へ変換する。
    const auto cos_yaw = std::cos(start_pose.yaw);
    const auto sin_yaw = std::sin(start_pose.yaw);
    if (step.move_to_pose.has_position) {
      target.x = start_pose.x + cos_yaw * step.move_to_pose.position.x - sin_yaw * step.move_to_pose.position.y;
      target.y = start_pose.y + sin_yaw * step.move_to_pose.position.x + cos_yaw * step.move_to_pose.position.y;
    }
    if (step.move_to_pose.has_yaw) {
      target.yaw = normalize_angle(start_pose.yaw + step.move_to_pose.yaw.yaw);
    }
    return target;
  }

  bool wait_for_odometry()
  {
    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok() && !latest_pose_) {
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    return latest_pose_.has_value();
  }

  void clamp_linear_velocity(double & linear_x, double & linear_y) const
  {
    const auto speed = std::hypot(linear_x, linear_y);
    if (speed <= max_linear_speed_) {
      return;
    }

    const auto scale = max_linear_speed_ / speed;
    linear_x *= scale;
    linear_y *= scale;
  }

  void publish_twist(const double linear_x, const double linear_y, const double angular_z)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.angular.z = angular_z;
    publisher_->publish(twist);
  }

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string scenario_file_;
  double start_delay_sec_{};
  double publish_rate_{};
  double max_linear_speed_{};
  double max_angular_speed_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::optional<PlanarPose> latest_pose_;
  ScenarioDefinition scenario_;
};

}  // namespace ai_ship_robot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ai_ship_robot_gazebo::ScriptedDrive> node;

  try {
    node = std::make_shared<ai_ship_robot_gazebo::ScriptedDrive>();
    node->run();
  } catch (const std::exception & exc) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "scripted_drive failed: %s", exc.what());
    } else {
      fprintf(stderr, "scripted_drive failed: %s\n", exc.what());
    }
    if (node) {
      node->stop();
    }
    rclcpp::shutdown();
    return 1;
  }

  if (node) {
    node->stop();
  }
  rclcpp::shutdown();
  return 0;
}
