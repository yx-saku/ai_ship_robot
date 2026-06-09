#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace ai_ship_robot_slam
{
namespace
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;

double seconds_between(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return std::abs((lhs - rhs).seconds());
}
}  // namespace

class DualLidarPointCloudFusionNode : public rclcpp::Node
{
public:
  DualLidarPointCloudFusionNode()
  : Node("dual_lidar_pointcloud_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // SLAM入力の差し替えを容易にするため、topicやframeはすべてパラメータ化する。
    left_points_topic_ = declare_parameter<std::string>("left_points_topic", "/left_lidar/points");
    right_points_topic_ = declare_parameter<std::string>("right_points_topic", "/right_lidar/points");
    output_points_topic_ = declare_parameter<std::string>("output_points_topic", "/slam/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    max_stamp_delta_sec_ = declare_parameter<double>("max_stamp_delta_sec", 0.05);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.0);

    // センサ点群は欠落を避けるためSensorDataQoSを使い、出力もSLAM入力として同じQoSにする。
    const auto qos = rclcpp::SensorDataQoS();
    fused_points_pub_ = create_publisher<PointCloud2>(output_points_topic_, qos);
    left_points_sub_.subscribe(this, left_points_topic_, qos.get_rmw_qos_profile());
    right_points_sub_.subscribe(this, right_points_topic_, qos.get_rmw_qos_profile());

    // 2台LiDARは完全同時刻とは限らないため、許容幅を持つ近似同期で結合する。
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(std::max(1, sync_queue_size_)), left_points_sub_, right_points_sub_);
    sync_->registerCallback(
      std::bind(&DualLidarPointCloudFusionNode::handle_points, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Fuse point clouds: left=%s right=%s output=%s target_frame=%s",
      left_points_topic_.c_str(), right_points_topic_.c_str(), output_points_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void handle_points(const PointCloud2::ConstSharedPtr & left, const PointCloud2::ConstSharedPtr & right)
  {
    const rclcpp::Time left_stamp(left->header.stamp);
    const rclcpp::Time right_stamp(right->header.stamp);

    // 同期ずれが大きい点群はSLAMへ渡すと歪みになるため、結合せずに捨てる。
    if (seconds_between(left_stamp, right_stamp) > max_stamp_delta_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip point clouds because timestamp delta is %.3f sec",
        seconds_between(left_stamp, right_stamp));
      return;
    }

    PointCloud2 transformed_left;
    PointCloud2 transformed_right;
    if (!transform_cloud(*left, transformed_left) || !transform_cloud(*right, transformed_right)) {
      return;
    }

    // TF変換済みの2台分をPCLへ変換し、glimへ渡す1つの点群にまとめる。
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformed_left, *left_cloud);
    pcl::fromROSMsg(transformed_right, *right_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    fused_cloud->reserve(left_cloud->size() + right_cloud->size());
    *fused_cloud += *left_cloud;
    *fused_cloud += *right_cloud;

    // CPU負荷を抑えたい環境ではvoxel downsampleを有効化し、未指定なら原点群を保持する。
    if (voxel_leaf_size_ > 0.0 && !fused_cloud->empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(fused_cloud);
      voxel_grid.setLeafSize(
        static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      voxel_grid.filter(*filtered_cloud);
      fused_cloud = filtered_cloud;
    }

    PointCloud2 output;
    pcl::toROSMsg(*fused_cloud, output);
    output.header.frame_id = target_frame_;
    output.header.stamp = left_stamp <= right_stamp ? left->header.stamp : right->header.stamp;
    fused_points_pub_->publish(output);
  }

  bool transform_cloud(const PointCloud2 & input, PointCloud2 & output)
  {
    if (input.header.frame_id == target_frame_) {
      output = input;
      return true;
    }

    try {
      // 各LiDARの固定TFを使い、シミュレーション/実機どちらでも同じ統合処理にする。
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, input.header.frame_id, input.header.stamp,
        tf2::durationFromSec(tf_timeout_sec_));
      tf2::doTransform(input, output, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform point cloud from %s to %s: %s",
        input.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::string left_points_topic_;
  std::string right_points_topic_;
  std::string output_points_topic_;
  std::string target_frame_;
  int sync_queue_size_;
  double max_stamp_delta_sec_;
  double tf_timeout_sec_;
  double voxel_leaf_size_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<PointCloud2> left_points_sub_;
  message_filters::Subscriber<PointCloud2> right_points_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<PointCloud2>::SharedPtr fused_points_pub_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::DualLidarPointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
