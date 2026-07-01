#ifndef AI_SHIP_ROBOT_GAZEBO__CMD_VEL_SLOPE_ADAPTER_HPP_
#define AI_SHIP_ROBOT_GAZEBO__CMD_VEL_SLOPE_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <optional>
#include <string>

namespace ai_ship_robot_gazebo
{

class CmdVelSlopeAdapter : public rclcpp::Node
{
public:
  CmdVelSlopeAdapter();
  explicit CmdVelSlopeAdapter(const rclcpp::NodeOptions & options);
  void handle_cmd_vel_for_test(const geometry_msgs::msg::Twist & message);

private:
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);
  geometry_msgs::msg::Twist apply_acceleration_limits(const geometry_msgs::msg::Twist & command);
  double limit_axis_rate(
    const double target_value,
    const double previous_value,
    const double max_acceleration,
    const double dt) const;
  void maybe_log_lateral_warning(const geometry_msgs::msg::Twist & adapted, const double original_linear_y);
  void maybe_log_limit_warning(
    const geometry_msgs::msg::Twist & input,
    const geometry_msgs::msg::Twist & adapted,
    const bool linear_limited,
    const bool angular_limited);
  bool should_emit_warning(const std::string & signature);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Time last_warning_time_;
  std::string last_warning_signature_;
  double warning_interval_sec_{};
  bool warn_on_lateral_command_{};
  double max_linear_acceleration_mps2_{};
  double max_angular_acceleration_radps2_{};
  std::optional<geometry_msgs::msg::Twist> last_output_command_;
  rclcpp::Time last_output_time_;
};

}  // namespace ai_ship_robot_gazebo

#endif  // AI_SHIP_ROBOT_GAZEBO__CMD_VEL_SLOPE_ADAPTER_HPP_
