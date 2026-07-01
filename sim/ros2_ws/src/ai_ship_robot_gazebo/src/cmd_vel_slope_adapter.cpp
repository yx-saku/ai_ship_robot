#include "ai_ship_robot_gazebo/cmd_vel_slope_adapter.hpp"

#include "ai_ship_robot_gazebo/drive_limits.hpp"

#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace ai_ship_robot_gazebo
{

CmdVelSlopeAdapter::CmdVelSlopeAdapter()
: CmdVelSlopeAdapter(rclcpp::NodeOptions{})
{
}

CmdVelSlopeAdapter::CmdVelSlopeAdapter(const rclcpp::NodeOptions & options)
: Node("cmd_vel_slope_adapter", options)
{
  declare_parameter<std::string>("input_topic", "/cmd_vel");
  declare_parameter<std::string>("output_topic", "/cmd_vel_slope_limited");
  declare_parameter<double>("warning_interval_sec", 5.0);
  declare_parameter<bool>("warn_on_lateral_command", true);
  declare_parameter<double>("max_linear_acceleration_mps2", 1000.0);
  declare_parameter<double>("max_angular_acceleration_radps2", 1000.0);

  const auto input_topic = get_parameter("input_topic").as_string();
  const auto output_topic = get_parameter("output_topic").as_string();
  warning_interval_sec_ = get_parameter("warning_interval_sec").as_double();
  warn_on_lateral_command_ = get_parameter("warn_on_lateral_command").as_bool();
  max_linear_acceleration_mps2_ = get_parameter("max_linear_acceleration_mps2").as_double();
  max_angular_acceleration_radps2_ = get_parameter("max_angular_acceleration_radps2").as_double();

  // 外部 /cmd_vel の見かけを維持しつつ、坂道検証用の制限を内部topicへ集約する。
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10));
  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    input_topic,
    rclcpp::QoS(10),
    std::bind(&CmdVelSlopeAdapter::handle_cmd_vel, this, std::placeholders::_1));
}

void CmdVelSlopeAdapter::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
{
  auto adapted = *message;
  const double original_linear_y = adapted.linear.y;

  // 坂道検証モードでは横移動を一元的に無効化し、入力元ごとの差異を作らない。
  if (std::abs(adapted.linear.y) > 1e-9) {
    adapted.linear.y = 0.0;
  }

  // 前転しやすい急加速を抑えるため、横成分を落とした後の前進・旋回指令を対称レートで制限する。
  adapted = apply_acceleration_limits(adapted);

  // controller手前でも同じ上限制約を再適用し、外部publish経路でも最終入力を統一する。
  const auto limited = apply_drive_limits(adapted.linear.x, adapted.linear.y, adapted.angular.z);
  adapted.linear.x = limited.command.linear_x;
  adapted.linear.y = limited.command.linear_y;
  adapted.angular.z = limited.command.angular_z;

  maybe_log_lateral_warning(adapted, original_linear_y);
  maybe_log_limit_warning(*message, adapted, limited.linear_limited, limited.angular_limited);

  publisher_->publish(adapted);
}

geometry_msgs::msg::Twist CmdVelSlopeAdapter::apply_acceleration_limits(
  const geometry_msgs::msg::Twist & command)
{
  auto limited = command;
  const auto now = get_clock()->now();

  // 初回入力では現在値からの段階加速を開始し、ステップ入力をそのまま下流へ流さない。
  if (!last_output_command_.has_value()) {
    last_output_command_ = command;
    last_output_time_ = now;
    return command;
  }

  const double dt = (now - last_output_time_).seconds();
  if (dt <= 0.0) {
    // 時刻が進まない場合は安全側として前回出力を維持し、テストやsim_time切替直後の不安定化を避ける。
    return *last_output_command_;
  }

  limited.linear.x = limit_axis_rate(
    command.linear.x,
    last_output_command_->linear.x,
    max_linear_acceleration_mps2_,
    dt);
  limited.angular.z = limit_axis_rate(
    command.angular.z,
    last_output_command_->angular.z,
    max_angular_acceleration_radps2_,
    dt);

  last_output_command_ = limited;
  last_output_time_ = now;
  return limited;
}

double CmdVelSlopeAdapter::limit_axis_rate(
  const double target_value,
  const double previous_value,
  const double max_acceleration,
  const double dt) const
{
  if (max_acceleration <= 0.0) {
    return previous_value;
  }

  const double max_delta = max_acceleration * dt;
  const double requested_delta = target_value - previous_value;
  return previous_value + std::clamp(requested_delta, -max_delta, max_delta);
}

void CmdVelSlopeAdapter::handle_cmd_vel_for_test(const geometry_msgs::msg::Twist & message)
{
  handle_cmd_vel(std::make_shared<geometry_msgs::msg::Twist>(message));
}

void CmdVelSlopeAdapter::maybe_log_lateral_warning(
  const geometry_msgs::msg::Twist & adapted,
  const double original_linear_y)
{
  if (!warn_on_lateral_command_ || std::abs(original_linear_y) <= 1e-9) {
    return;
  }

  std::ostringstream signature;
  signature << "lateral:" << original_linear_y;
  if (!should_emit_warning(signature.str())) {
    return;
  }

  // 横移動無効化は入力値を明示し、斜め入力が前進だけへ変わることを追跡しやすくする。
  RCLCPP_WARN(
    get_logger(),
    "Ignoring cmd_vel.linear.y=%.3f because lateral motion is disabled in slope validation mode. Output=(%.3f, %.3f, %.3f)",
    original_linear_y,
    adapted.linear.x,
    adapted.linear.y,
    adapted.angular.z);
}

void CmdVelSlopeAdapter::maybe_log_limit_warning(
  const geometry_msgs::msg::Twist & input,
  const geometry_msgs::msg::Twist & adapted,
  const bool linear_limited,
  const bool angular_limited)
{
  if (!linear_limited && !angular_limited) {
    return;
  }

  std::ostringstream signature;
  signature << "limit:"
            << input.linear.x << ","
            << input.linear.y << ","
            << input.angular.z << "->"
            << adapted.linear.x << ","
            << adapted.linear.y << ","
            << adapted.angular.z;
  if (!should_emit_warning(signature.str())) {
    return;
  }

  // 速度制限は入力とcontroller向け出力の差分を残し、手動・自動・外部publishの挙動差を診断しやすくする。
  RCLCPP_WARN(
    get_logger(),
    "Drive command limited in slope adapter: input=(%.3f, %.3f, %.3f) output=(%.3f, %.3f, %.3f)",
    input.linear.x,
    input.linear.y,
    input.angular.z,
    adapted.linear.x,
    adapted.linear.y,
    adapted.angular.z);
}

bool CmdVelSlopeAdapter::should_emit_warning(const std::string & signature)
{
  const auto now = get_clock()->now();
  if (
    last_warning_time_.nanoseconds() != 0 &&
    (now - last_warning_time_).seconds() < warning_interval_sec_ &&
    last_warning_signature_ == signature)
  {
    return false;
  }

  last_warning_time_ = now;
  last_warning_signature_ = signature;
  return true;
}

}  // namespace ai_ship_robot_gazebo

#ifndef AI_SHIP_ROBOT_GAZEBO_BUILDING_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_gazebo::CmdVelSlopeAdapter>());
  rclcpp::shutdown();
  return 0;
}
#endif
