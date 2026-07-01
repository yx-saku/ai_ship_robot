#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ai_ship_robot_gazebo
{

class SlopeDriveBridge : public rclcpp::Node
{
public:
  SlopeDriveBridge()
  : Node("slope_drive_bridge")
  {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_slope_limited");
    declare_parameter<std::string>("controller_command_topic", "/slope_drive_controller/commands");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("odometry_frame", "odom");
    declare_parameter<std::string>("robot_base_frame", "base_footprint");
    declare_parameter<bool>("publish_odom_tf", true);
    declare_parameter<double>("wheel_radius", 0.04);
    declare_parameter<double>("track_width", 0.54);
    declare_parameter<std::vector<std::string>>("left_wheels", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("right_wheels", std::vector<std::string>{});

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    controller_command_topic_ = get_parameter("controller_command_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    odometry_frame_ = get_parameter("odometry_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    publish_odom_tf_ = get_parameter("publish_odom_tf").as_bool();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    left_wheels_ = get_parameter("left_wheels").as_string_array();
    right_wheels_ = get_parameter("right_wheels").as_string_array();
    axle_track_ = std::max(track_width_, 1e-6);

    // 外部速度指令は左右輪の角速度列へ落とし込み、同側前後輪へ同じ値を配る。
    controller_command_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(controller_command_topic_, rclcpp::QoS(10));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10), std::bind(&SlopeDriveBridge::handle_cmd_vel, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(50),
      std::bind(&SlopeDriveBridge::handle_joint_states, this, std::placeholders::_1));

    if (publish_odom_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double linear_x = message->linear.x;
    const double angular_z = message->angular.z;
    const double half_track = axle_track_ / 2.0;
    const double left_linear = linear_x - angular_z * half_track;
    const double right_linear = linear_x + angular_z * half_track;
    const double left_velocity = left_linear / wheel_radius_;
    const double right_velocity = right_linear / wheel_radius_;

    std_msgs::msg::Float64MultiArray command;
    command.data.reserve(left_wheels_.size() + right_wheels_.size());
    for (std::size_t index = 0; index < left_wheels_.size(); ++index) {
      command.data.push_back(left_velocity);
    }
    for (std::size_t index = 0; index < right_wheels_.size(); ++index) {
      command.data.push_back(right_velocity);
    }
    controller_command_publisher_->publish(command);
  }

  void handle_joint_states(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->name.empty() || message->velocity.size() != message->name.size()) {
      return;
    }

    std::unordered_map<std::string, double> velocities;
    velocities.reserve(message->name.size());
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      velocities.emplace(message->name[index], message->velocity[index]);
    }

    const auto left_velocity = average_group_velocity(left_wheels_, velocities);
    const auto right_velocity = average_group_velocity(right_wheels_, velocities);
    if (!left_velocity.has_value() || !right_velocity.has_value()) {
      return;
    }

    rclcpp::Time stamp = now();
    if (!(message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0)) {
      stamp = rclcpp::Time(message->header.stamp);
    }
    if (last_joint_state_stamp_.nanoseconds() == 0) {
      last_joint_state_stamp_ = stamp;
      return;
    }

    const double dt = (stamp - last_joint_state_stamp_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_joint_state_stamp_ = stamp;

    // wheel state から差動二輪近似の odom を積分し、既存 /odom インタフェースを維持する。
    const double left_linear = left_velocity.value() * wheel_radius_;
    const double right_linear = right_velocity.value() * wheel_radius_;
    const double linear_x = (left_linear + right_linear) / 2.0;
    const double angular_z = (right_linear - left_linear) / axle_track_;
    const double heading_mid = yaw_ + angular_z * dt / 2.0;

    x_ += linear_x * std::cos(heading_mid) * dt;
    y_ += linear_x * std::sin(heading_mid) * dt;
    yaw_ += angular_z * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation = tf2::toMsg(orientation);
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.angular.z = angular_z;
    odom_publisher_->publish(odom);

    if (publish_odom_tf_ && tf_broadcaster_ != nullptr) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = robot_base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::optional<double> average_group_velocity(
    const std::vector<std::string> & joint_names,
    const std::unordered_map<std::string, double> & velocities) const
  {
    double sum = 0.0;
    std::size_t count = 0;
    for (const auto & joint_name : joint_names) {
      const auto iter = velocities.find(joint_name);
      if (iter == velocities.end()) {
        continue;
      }
      sum += iter->second;
      ++count;
    }
    if (count == 0) {
      return std::nullopt;
    }
    return sum / static_cast<double>(count);
  }

  std::string cmd_vel_topic_;
  std::string controller_command_topic_;
  std::string joint_states_topic_;
  std::string odom_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  bool publish_odom_tf_{};
  double wheel_radius_{};
  double track_width_{};
  double axle_track_{};
  double x_{};
  double y_{};
  double yaw_{};
  rclcpp::Time last_joint_state_stamp_;
  std::vector<std::string> left_wheels_;
  std::vector<std::string> right_wheels_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr controller_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace ai_ship_robot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_gazebo::SlopeDriveBridge>());
  rclcpp::shutdown();
  return 0;
}
