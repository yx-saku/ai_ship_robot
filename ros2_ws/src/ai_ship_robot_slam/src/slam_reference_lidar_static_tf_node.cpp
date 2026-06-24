// Copyright 2026 AI Ship Robot Developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ai_ship_robot_slam
{
namespace
{

bool is_finite(const double value)
{
  return std::isfinite(value);
}

bool is_finite_quaternion(const tf2::Quaternion & quaternion)
{
  return is_finite(quaternion.x()) && is_finite(quaternion.y()) && is_finite(quaternion.z()) &&
         is_finite(quaternion.w()) && quaternion.length2() > 1.0e-18;
}

geometry_msgs::msg::Vector3 make_translation_message(const tf2::Vector3 & translation)
{
  geometry_msgs::msg::Vector3 message;
  message.x = translation.x();
  message.y = translation.y();
  message.z = translation.z();
  return message;
}

geometry_msgs::msg::Quaternion make_quaternion_message(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

tf2::Transform transform_from_message(const geometry_msgs::msg::Transform & message)
{
  tf2::Quaternion rotation(
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w);
  if (!is_finite_quaternion(rotation)) {
    throw std::runtime_error("reference LiDAR transform has an invalid rotation quaternion");
  }
  rotation.normalize();

  const tf2::Vector3 translation(
    message.translation.x, message.translation.y, message.translation.z);
  if (!is_finite(translation.x()) || !is_finite(translation.y()) || !is_finite(translation.z())) {
    throw std::runtime_error("reference LiDAR transform has an invalid translation");
  }
  return tf2::Transform(rotation, translation);
}

double projected_x_axis_yaw(const tf2::Matrix3x3 & rotation)
{
  const double x_axis_x = rotation[0][0];
  const double x_axis_y = rotation[1][0];

  // 基準LiDARの+X軸を水平面へ射影し、SLAM初期frameにはyaw成分だけを反映する。
  if (std::hypot(x_axis_x, x_axis_y) <= 1.0e-9) {
    return 0.0;
  }
  return std::atan2(x_axis_y, x_axis_x);
}

}  // namespace

class SlamReferenceLidarStaticTfNode : public rclcpp::Node
{
public:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;

  SlamReferenceLidarStaticTfNode()
  : Node("slam_reference_lidar_static_tf_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this),
    static_broadcaster_(this)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    reference_custom_topic_ = declare_parameter<std::string>(
      "reference_custom_topic", "/lidar1/livox/lidar");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    lidar_init_frame_ = declare_parameter<std::string>("lidar_init_frame", "lidar_init");
    lidar_odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", "lidar_odom");
    publish_map_to_lidar_init_ = declare_parameter<bool>("publish_map_to_lidar_init", true);
    const double lookup_period_sec = declare_parameter<double>("lookup_period_sec", 0.2);

    validate_parameters(lookup_period_sec);

    // 基準LiDARのCustomMsgから実frameを取得し、設定ファイルとの二重管理を避ける。
    reference_cloud_subscription_ = create_subscription<CustomMsg>(
      reference_custom_topic_, rclcpp::SensorDataQoS(),
      [this](const CustomMsg::SharedPtr message) {handle_reference_cloud(message);});

    // CustomMsgと/tf_staticの到着順が前後しても、両方そろった時点でTFを確定する。
    const auto lookup_period = std::chrono::duration<double>(lookup_period_sec);
    lookup_timer_ = create_wall_timer(lookup_period, [this]() {try_publish_static_transforms();});
  }

private:
  void validate_parameters(const double lookup_period_sec) const
  {
    if (base_frame_.empty() || reference_custom_topic_.empty() || map_frame_.empty() ||
      lidar_init_frame_.empty() || lidar_odom_frame_.empty())
    {
      throw std::invalid_argument("frame and topic parameters must not be empty");
    }
    if (!std::isfinite(lookup_period_sec) || lookup_period_sec <= 0.0) {
      throw std::invalid_argument("lookup_period_sec must be greater than 0");
    }
  }

  void handle_reference_cloud(const CustomMsg::SharedPtr message)
  {
    if (published_ || message == nullptr) {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for non-empty frame_id on reference CustomMsg topic %s.",
        reference_custom_topic_.c_str());
      return;
    }

    // 最初に観測した基準LiDAR frameを採用し、以後は同じframeでstatic TFを確定する。
    if (reference_lidar_frame_.empty()) {
      reference_lidar_frame_ = message->header.frame_id;
      RCLCPP_INFO(
        get_logger(), "Reference LiDAR frame resolved from %s: %s",
        reference_custom_topic_.c_str(), reference_lidar_frame_.c_str());
    }
    try_publish_static_transforms();
  }

  void try_publish_static_transforms()
  {
    if (published_) {
      // ROS graph discovery遅延で初回/tf_staticを取り逃がすlistenerへ、確定済みTFを再送する。
      static_broadcaster_.sendTransform(published_transforms_);
      return;
    }
    if (reference_lidar_frame_.empty()) {
      log_waiting_for_reference_cloud();
      return;
    }

    try {
      const auto base_to_lidar_message = tf_buffer_.lookupTransform(
        base_frame_, reference_lidar_frame_, tf2::TimePointZero);
      const auto base_to_lidar = transform_from_message(base_to_lidar_message.transform);
      publish_static_transforms(base_to_lidar);
    } catch (const tf2::TransformException & error) {
      log_waiting_for_reference_tf(error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to publish SLAM static TF: %s", error.what());
      throw;
    }
  }

  void log_waiting_for_reference_tf(const std::string & message)
  {
    ++lookup_attempt_count_;
    if (lookup_attempt_count_ == 1 || lookup_attempt_count_ % 25 == 0) {
      RCLCPP_INFO(
        get_logger(), "Waiting for static TF %s -> %s: %s", base_frame_.c_str(),
        reference_lidar_frame_.c_str(), message.c_str());
    }
  }

  void log_waiting_for_reference_cloud()
  {
    ++reference_cloud_wait_count_;
    if (reference_cloud_wait_count_ == 1 || reference_cloud_wait_count_ % 25 == 0) {
      RCLCPP_INFO(
        get_logger(), "Waiting for reference CustomMsg on %s to resolve LiDAR frame.",
        reference_custom_topic_.c_str());
    }
  }

  void publish_static_transforms(const tf2::Transform & base_to_lidar)
  {
    const auto stamp = get_clock()->now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(publish_map_to_lidar_init_ ? 2 : 1);

    if (publish_map_to_lidar_init_) {
      // localization時はlocalizerがmap->lidar_initを所有するため、競合するstatic TFだけを止める。
      const tf2::Matrix3x3 base_to_lidar_rotation(base_to_lidar.getRotation());
      const double lidar_init_yaw = projected_x_axis_yaw(base_to_lidar_rotation);
      tf2::Quaternion lidar_init_rotation;
      lidar_init_rotation.setRPY(0.0, 0.0, lidar_init_yaw);
      lidar_init_rotation.normalize();

      geometry_msgs::msg::TransformStamped map_to_lidar_init;
      map_to_lidar_init.header.stamp = stamp;
      map_to_lidar_init.header.frame_id = map_frame_;
      map_to_lidar_init.child_frame_id = lidar_init_frame_;
      map_to_lidar_init.transform.translation = make_translation_message(base_to_lidar.getOrigin());
      map_to_lidar_init.transform.rotation = make_quaternion_message(lidar_init_rotation);
      transforms.push_back(map_to_lidar_init);
    }

    // LIO-SAMの推定LiDAR poseからbase_footprintへ接続し、URDF側のlink treeへつなぐ。
    const tf2::Transform lidar_to_base = base_to_lidar.inverse();
    geometry_msgs::msg::TransformStamped lidar_odom_to_base;
    lidar_odom_to_base.header.stamp = stamp;
    lidar_odom_to_base.header.frame_id = lidar_odom_frame_;
    lidar_odom_to_base.child_frame_id = base_frame_;
    lidar_odom_to_base.transform.translation = make_translation_message(lidar_to_base.getOrigin());
    lidar_odom_to_base.transform.rotation = make_quaternion_message(lidar_to_base.getRotation());
    transforms.push_back(lidar_odom_to_base);

    static_broadcaster_.sendTransform(transforms);
    published_transforms_ = transforms;
    published_ = true;
    if (publish_map_to_lidar_init_) {
      RCLCPP_INFO(
        get_logger(),
        "Published SLAM static TFs from %s -> %s: %s -> %s and %s -> %s",
        base_frame_.c_str(), reference_lidar_frame_.c_str(), map_frame_.c_str(),
        lidar_init_frame_.c_str(), lidar_odom_frame_.c_str(), base_frame_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Published SLAM static TF from %s -> %s: %s -> %s; skip %s -> %s",
        base_frame_.c_str(), reference_lidar_frame_.c_str(), lidar_odom_frame_.c_str(),
        base_frame_.c_str(), map_frame_.c_str(), lidar_init_frame_.c_str());
    }
  }

  std::string base_frame_;
  std::string reference_custom_topic_;
  std::string reference_lidar_frame_;
  std::string map_frame_;
  std::string lidar_init_frame_;
  std::string lidar_odom_frame_;
  bool publish_map_to_lidar_init_{true};
  std::vector<geometry_msgs::msg::TransformStamped> published_transforms_;
  bool published_{false};
  std::size_t lookup_attempt_count_{0};
  std::size_t reference_cloud_wait_count_{0};
  rclcpp::TimerBase::SharedPtr lookup_timer_;
  rclcpp::Subscription<CustomMsg>::SharedPtr reference_cloud_subscription_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ai_ship_robot_slam::SlamReferenceLidarStaticTfNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "slam_reference_lidar_static_tf_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
