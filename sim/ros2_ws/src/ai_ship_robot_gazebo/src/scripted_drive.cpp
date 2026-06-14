#include "ai_ship_robot_gazebo/scripted_drive_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace ai_ship_robot_gazebo
{

class ScriptedDrive : public rclcpp::Node
{
public:
  ScriptedDrive()
  : Node("scripted_drive")
  {
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<std::string>("scenario_file", "");
    declare_parameter<double>("start_delay_sec", 0.0);
    declare_parameter<bool>("loop", false);
    declare_parameter<double>("publish_rate", 10.0);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    scenario_file_ = get_parameter("scenario_file").as_string();
    start_delay_sec_ = get_parameter("start_delay_sec").as_double();
    loop_enabled_ = get_parameter("loop").as_bool();
    publish_rate_ = get_parameter("publish_rate").as_double();

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    steps_ = load_scenario_file(scenario_file_);
  }

  void run()
  {
    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be > 0.");
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
    bool keep_running = true;
    while (rclcpp::ok() && keep_running) {
      RCLCPP_INFO(get_logger(), "Starting scripted drive scenario");
      run_steps(period);
      keep_running = loop_enabled_;
    }
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
    for (std::size_t index = 0; rclcpp::ok() && index < steps_.size(); ++index) {
      const auto & step = steps_[index];
      const auto active_step_time = wait_for_active_sim_time();
      if (!active_step_time) {
        return;
      }
      auto step_start_time = *active_step_time;
      auto deadline = step_start_time + rclcpp::Duration::from_seconds(step.duration_sec);
      RCLCPP_INFO(get_logger(), "Step %zu: %s", index + 1, format_step_command(step).c_str());

      // publish周期はwall sleep相当、終了判定は有効なGazebo時刻からの経過時間で行う。
      rclcpp::WallRate rate(1.0 / period.count());
      while (rclcpp::ok()) {
        publish_twist(step.linear_x, step.linear_y, step.angular_z);
        rclcpp::spin_some(shared_from_this());
        const auto current_time = get_clock()->now();
        if (current_time.nanoseconds() == 0) {
          rate.sleep();
          continue;
        }
        if (current_time < step_start_time) {
          step_start_time = current_time;
          deadline = current_time + rclcpp::Duration::from_seconds(step.duration_sec);
        }
        if (current_time >= deadline) {
          break;
        }
        rate.sleep();
      }
      stop();
      RCLCPP_INFO(get_logger(), "Step %zu: stop", index + 1);
    }
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
  std::string scenario_file_;
  double start_delay_sec_{};
  bool loop_enabled_{};
  double publish_rate_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::vector<ScenarioStep> steps_;
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
