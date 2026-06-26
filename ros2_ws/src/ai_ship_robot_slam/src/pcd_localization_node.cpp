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

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace ai_ship_robot_slam
{
namespace
{
using Odometry = nav_msgs::msg::Odometry;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

enum class LocalizationState
{
  UNINITIALIZED,
  TRACKING,
  LOST,
  RELOCALIZING,
};

struct CloudBounds
{
  float min_x{std::numeric_limits<float>::max()};
  float max_x{std::numeric_limits<float>::lowest()};
  float min_y{std::numeric_limits<float>::max()};
  float max_y{std::numeric_limits<float>::lowest()};
};

struct ScanContextDescriptor
{
  int rings{};
  int sectors{};
  std::vector<unsigned char> occupied;
  std::size_t occupied_count{};
};

struct MapContext
{
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  CloudT::Ptr cloud{new CloudT};
  ScanContextDescriptor descriptor;
};

struct CandidateMatch
{
  const MapContext * context{};
  double score{std::numeric_limits<double>::infinity()};
  int sector_shift{};
  double yaw{};
};

struct RegistrationResult
{
  bool converged{};
  double fitness{std::numeric_limits<double>::infinity()};
  tf2::Transform target_from_source;
};

struct DistanceScore
{
  bool valid{};
  double score{std::numeric_limits<double>::infinity()};
  double mean_distance{std::numeric_limits<double>::infinity()};
  double hit_ratio{};
  double far_ratio{};
  double out_of_bounds_ratio{1.0};
  std::size_t evaluated_count{};
};

struct InitialGuessCandidate
{
  CandidateMatch candidate;
  tf2::Transform map_from_lidar_init;
  DistanceScore distance_score;
  bool refined_with_distance{};
};

struct InitialResult
{
  CandidateMatch candidate;
  RegistrationResult registration;
  double coarse_fitness{std::numeric_limits<double>::infinity()};
  bool refined_with_gicp{};
  DistanceScore distance_score;
  bool refined_with_distance{};
};

struct DistanceFieldMap
{
  bool ready{};
  double resolution{};
  double origin_x{};
  double origin_y{};
  int width{};
  int height{};
  std::size_t occupied_cells{};
  cv::Mat distance_meters;
};

struct CloudFeatureMetrics
{
  std::size_t filtered_points{};
  std::size_t xy_cells{};
  double linearity{1.0};
  double planarity{};
  double scattering{};
  bool passed{};
};

struct StampedOdometry
{
  rclcpp::Time stamp;
  tf2::Transform lidar_init_from_lidar_odom;
};

double elapsed_milliseconds(
  const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string state_to_string(const LocalizationState state)
{
  switch (state) {
    case LocalizationState::UNINITIALIZED:
      return "UNINITIALIZED";
    case LocalizationState::TRACKING:
      return "TRACKING";
    case LocalizationState::LOST:
      return "LOST";
    case LocalizationState::RELOCALIZING:
      return "RELOCALIZING";
  }
  return "UNKNOWN";
}

bool is_finite_transform(const tf2::Transform & transform)
{
  const auto & origin = transform.getOrigin();
  const auto & rotation = transform.getRotation();
  return std::isfinite(origin.x()) && std::isfinite(origin.y()) && std::isfinite(origin.z()) &&
         std::isfinite(rotation.x()) && std::isfinite(rotation.y()) &&
         std::isfinite(rotation.z()) && std::isfinite(rotation.w()) && rotation.length2() > 1.0e-18;
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (!std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
    !std::isfinite(rotation.z()) || !std::isfinite(rotation.w()) || rotation.length2() <= 1.0e-18)
  {
    throw std::runtime_error("odometry pose has an invalid quaternion");
  }
  rotation.normalize();

  tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(rotation, translation);
}

Eigen::Matrix4f transform_to_eigen(const tf2::Transform & transform)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  const auto & basis = transform.getBasis();
  const auto & origin = transform.getOrigin();

  // tf2の行列要素をPCL registrationが扱う同次変換へ写し、frame計算を単一表現にそろえる。
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix(row, col) = static_cast<float>(basis[row][col]);
    }
  }
  matrix(0, 3) = static_cast<float>(origin.x());
  matrix(1, 3) = static_cast<float>(origin.y());
  matrix(2, 3) = static_cast<float>(origin.z());
  return matrix;
}

tf2::Transform eigen_to_transform(const Eigen::Matrix4f & matrix)
{
  tf2::Matrix3x3 basis(
    matrix(0, 0), matrix(0, 1), matrix(0, 2),
    matrix(1, 0), matrix(1, 1), matrix(1, 2),
    matrix(2, 0), matrix(2, 1), matrix(2, 2));
  tf2::Quaternion rotation;
  basis.getRotation(rotation);
  rotation.normalize();

  tf2::Vector3 translation(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  return tf2::Transform(rotation, translation);
}

geometry_msgs::msg::TransformStamped make_transform_message(
  const tf2::Transform & transform, const rclcpp::Time & stamp, const std::string & parent_frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent_frame;
  message.child_frame_id = child_frame;
  message.transform.translation.x = transform.getOrigin().x();
  message.transform.translation.y = transform.getOrigin().y();
  message.transform.translation.z = transform.getOrigin().z();
  message.transform.rotation.x = transform.getRotation().x();
  message.transform.rotation.y = transform.getRotation().y();
  message.transform.rotation.z = transform.getRotation().z();
  message.transform.rotation.w = transform.getRotation().w();
  return message;
}

geometry_msgs::msg::PoseWithCovarianceStamped make_pose_message(
  const tf2::Transform & transform, const rclcpp::Time & stamp, const std::string & frame_id)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.pose.pose.position.x = transform.getOrigin().x();
  message.pose.pose.position.y = transform.getOrigin().y();
  message.pose.pose.position.z = transform.getOrigin().z();
  message.pose.pose.orientation.x = transform.getRotation().x();
  message.pose.pose.orientation.y = transform.getRotation().y();
  message.pose.pose.orientation.z = transform.getRotation().z();
  message.pose.pose.orientation.w = transform.getRotation().w();

  // 初期実装では共分散を固定値にし、Nav2連携時に品質指標から更新できる余地を残す。
  message.pose.covariance.fill(0.0);
  message.pose.covariance[0] = 0.05 * 0.05;
  message.pose.covariance[7] = 0.05 * 0.05;
  message.pose.covariance[14] = 0.10 * 0.10;
  message.pose.covariance[35] = 0.05 * 0.05;
  return message;
}

tf2::Transform make_planar_transform(
  const double x, const double y, const double z, const double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  rotation.normalize();
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

double yaw_from_transform(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

std::uint64_t make_cell_key(const int x, const int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
         static_cast<std::uint32_t>(y);
}

bool is_finite_point(const PointT & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

CloudT::Ptr remove_non_finite_points(const CloudT::ConstPtr & input)
{
  CloudT::Ptr output(new CloudT);
  output->reserve(input->size());
  for (const auto & point : input->points) {
    if (is_finite_point(point)) {
      output->push_back(point);
    }
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

CloudT::Ptr voxel_downsample(const CloudT::ConstPtr & input, const double leaf_size)
{
  if (leaf_size <= 0.0 || input->empty()) {
    CloudT::Ptr copy(new CloudT(*input));
    return copy;
  }

  // PCD保存粒度とは別に、localization用の点数だけをPCL VoxelGridで抑える。
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(input);
  voxel_grid.setLeafSize(
    static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
  CloudT::Ptr output(new CloudT);
  voxel_grid.filter(*output);
  return output;
}

CloudBounds compute_bounds(const CloudT::ConstPtr & cloud)
{
  CloudBounds bounds;
  for (const auto & point : cloud->points) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  return bounds;
}

CloudT::Ptr extract_cloud_neighborhood(
  const CloudT::ConstPtr & cloud, const Eigen::Vector3f & center, const double radius,
  const double z_radius)
{
  CloudT::Ptr neighborhood(new CloudT);
  const double radius_sq = radius * radius;
  neighborhood->reserve(std::min<std::size_t>(cloud->size(), 200000U));

  // Scan Context候補とNDT対象を同じ幾何条件で切り出し、密度だけを用途別に変える。
  for (const auto & point : cloud->points) {
    const double dx = static_cast<double>(point.x) - center.x();
    const double dy = static_cast<double>(point.y) - center.y();
    if (dx * dx + dy * dy > radius_sq) {
      continue;
    }
    if (z_radius > 0.0 && std::abs(static_cast<double>(point.z) - center.z()) > z_radius) {
      continue;
    }
    neighborhood->push_back(point);
  }
  neighborhood->width = static_cast<std::uint32_t>(neighborhood->size());
  neighborhood->height = 1;
  neighborhood->is_dense = true;
  return neighborhood;
}

CloudT::Ptr extract_cloud_neighborhood_with_z_band(
  const CloudT::ConstPtr & cloud, const Eigen::Vector3f & center, const double radius,
  const double z_min, const double z_max)
{
  CloudT::Ptr neighborhood(new CloudT);
  const double radius_sq = radius * radius;
  neighborhood->reserve(std::min<std::size_t>(cloud->size(), 200000U));

  // GICP特徴量判定では中心相対zではなく絶対z帯域を使い、床天井に偏った範囲を除外する。
  for (const auto & point : cloud->points) {
    const double dx = static_cast<double>(point.x) - center.x();
    const double dy = static_cast<double>(point.y) - center.y();
    if (dx * dx + dy * dy > radius_sq) {
      continue;
    }
    if (static_cast<double>(point.z) < z_min || static_cast<double>(point.z) > z_max) {
      continue;
    }
    neighborhood->push_back(point);
  }
  neighborhood->width = static_cast<std::uint32_t>(neighborhood->size());
  neighborhood->height = 1;
  neighborhood->is_dense = true;
  return neighborhood;
}

std::size_t count_xy_occupancy_cells(const CloudT::ConstPtr & cloud, const double cell_size)
{
  if (cell_size <= 0.0 || cloud->empty()) {
    return 0U;
  }

  std::unordered_set<std::uint64_t> cells;
  cells.reserve(cloud->size());

  // 点密度差の影響を減らすため、局所形状の広がりはXY occupancy cell数へ圧縮して評価する。
  for (const auto & point : cloud->points) {
    const int cell_x = static_cast<int>(std::floor(static_cast<double>(point.x) / cell_size));
    const int cell_y = static_cast<int>(std::floor(static_cast<double>(point.y) / cell_size));
    cells.insert(make_cell_key(cell_x, cell_y));
  }
  return cells.size();
}

std::tuple<double, double, double> compute_pca_shape_metrics(const CloudT::ConstPtr & cloud)
{
  if (cloud->size() < 3U) {
    return {1.0, 0.0, 0.0};
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : cloud->points) {
    centroid.x() += static_cast<double>(point.x);
    centroid.y() += static_cast<double>(point.y);
    centroid.z() += static_cast<double>(point.z);
  }
  centroid /= static_cast<double>(cloud->size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : cloud->points) {
    const Eigen::Vector3d centered(
      static_cast<double>(point.x) - centroid.x(),
      static_cast<double>(point.y) - centroid.y(),
      static_cast<double>(point.z) - centroid.z());
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(cloud->size());

  // 固有値比から線状性・平面性・散乱度を算出し、退化した局所形状を安全に判定する。
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return {1.0, 0.0, 0.0};
  }

  Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const double lambda1 = eigenvalues(2);
  const double lambda2 = eigenvalues(1);
  const double lambda3 = eigenvalues(0);
  if (!std::isfinite(lambda1) || lambda1 <= 1.0e-12) {
    return {1.0, 0.0, 0.0};
  }

  const double linearity = std::clamp((lambda1 - lambda2) / lambda1, 0.0, 1.0);
  const double planarity = std::clamp((lambda2 - lambda3) / lambda1, 0.0, 1.0);
  const double scattering = std::clamp(lambda3 / lambda1, 0.0, 1.0);
  return {linearity, planarity, scattering};
}

CloudFeatureMetrics evaluate_cloud_feature_metrics(
  const CloudT::ConstPtr & cloud, const double occupancy_cell_size, const std::size_t min_points,
  const std::size_t min_xy_cells, const double min_scattering, const double max_linearity)
{
  CloudFeatureMetrics metrics;
  metrics.filtered_points = cloud->size();
  metrics.xy_cells = count_xy_occupancy_cells(cloud, occupancy_cell_size);
  std::tie(metrics.linearity, metrics.planarity, metrics.scattering) =
    compute_pca_shape_metrics(cloud);

  // 点数・広がり・退化度を併用し、平坦面や線状物だけの局所範囲をGICP候補から外す。
  metrics.passed = metrics.filtered_points >= min_points && metrics.xy_cells >= min_xy_cells &&
    (metrics.scattering >= min_scattering || metrics.linearity <= max_linearity);
  return metrics;
}

std::size_t descriptor_index(
  const int ring_index, const int sector_index, const int sectors)
{
  return static_cast<std::size_t>(ring_index * sectors + sector_index);
}

ScanContextDescriptor make_scan_context_descriptor(
  const CloudT::ConstPtr & cloud, const Eigen::Vector3f & center, const int rings,
  const int sectors,
  const double max_radius)
{
  ScanContextDescriptor descriptor;
  descriptor.rings = rings;
  descriptor.sectors = sectors;
  descriptor.occupied.assign(static_cast<std::size_t>(rings * sectors), 0U);

  // 高さ値に依存しすぎないbinary occupancy descriptorで、初期候補を軽量に絞り込む。
  for (const auto & point : cloud->points) {
    const double dx = static_cast<double>(point.x) - center.x();
    const double dy = static_cast<double>(point.y) - center.y();
    const double radius = std::hypot(dx, dy);
    if (radius <= 1.0e-6 || radius > max_radius) {
      continue;
    }

    const auto ring_index = std::min(
      rings - 1, static_cast<int>(std::floor(radius / max_radius * static_cast<double>(rings))));
    double angle = std::atan2(dy, dx);
    if (angle < 0.0) {
      angle += kTwoPi;
    }
    const auto sector_index = std::min(
      sectors - 1, static_cast<int>(std::floor(angle / kTwoPi * static_cast<double>(sectors))));
    auto & occupied = descriptor.occupied[descriptor_index(ring_index, sector_index, sectors)];
    if (occupied == 0U) {
      occupied = 1U;
      ++descriptor.occupied_count;
    }
  }
  return descriptor;
}

double descriptor_distance(
  const ScanContextDescriptor & query, const ScanContextDescriptor & target, const int sector_shift)
{
  std::size_t intersection_count = 0;
  std::size_t union_count = 0;
  for (int ring = 0; ring < query.rings; ++ring) {
    for (int sector = 0; sector < query.sectors; ++sector) {
      const int shifted_sector = (sector + sector_shift) % query.sectors;
      const bool query_occupied =
        query.occupied[descriptor_index(ring, sector, query.sectors)] != 0U;
      const bool target_occupied =
        target.occupied[descriptor_index(ring, shifted_sector, target.sectors)] != 0U;
      if (query_occupied || target_occupied) {
        ++union_count;
      }
      if (query_occupied && target_occupied) {
        ++intersection_count;
      }
    }
  }

  if (union_count == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return 1.0 - static_cast<double>(intersection_count) / static_cast<double>(union_count);
}

std::pair<double, int> best_descriptor_alignment(
  const ScanContextDescriptor & query, const ScanContextDescriptor & target)
{
  double best_score = std::numeric_limits<double>::infinity();
  int best_shift = 0;
  for (int shift = 0; shift < query.sectors; ++shift) {
    const double score = descriptor_distance(query, target, shift);
    if (score < best_score) {
      best_score = score;
      best_shift = shift;
    }
  }
  return {best_score, best_shift};
}

}  // namespace

class PcdLocalizationNode : public rclcpp::Node
{
public:
  PcdLocalizationNode()
  : Node("pcd_localization_node"),
    tf_broadcaster_(this)
  {
    declare_and_validate_parameters();
    load_fixed_map();
    build_map_context_index();

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/localization/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    map_preview_publisher_ = create_publisher<PointCloud2>(
      map_preview_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    publish_map_preview();

    // LIO-SAMの短期odometryと登録済みscanを受け、固定PCD mapに対するposeだけを推定する。
    odometry_subscription_ = create_subscription<Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const Odometry::SharedPtr message) {handle_odometry(message);});
    cloud_subscription_ = create_subscription<PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const PointCloud2::SharedPtr message) {handle_cloud(message);});

    // 入力が来ない場合と初期化失敗を切り分けるため、状態だけを周期的に記録する。
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() {log_diagnostics();});

    RCLCPP_INFO(
      get_logger(),
      "PCD localization diagnostics: cloud_topic=%s odometry_topic=%s accumulation=%.3fs "
      "sc_leaf(map=%.3f scan=%.3f) initial_ndt_leaf(map=%.3f scan=%.3f) "
      "fine_gicp(enabled=%d candidates=%d leaf(map=%.3f scan=%.3f) threshold=%.3f "
      "early_accept=%d early_fitness=%.3f adaptive=%d radius=[%.3f,%.3f,%.3f,%.3f] "
      "feature_z=[%.3f,%.3f] min_points=(%zu,%zu) min_xy_cells=(%zu,%zu) cell=%.3f "
      "min_scattering=%.3f max_linearity=%.3f) "
      "distance_match(enabled=%d grid=%.3f max_dist=%.3f search_xy=%.3f step=%.3f "
      "yaw_range=%.3f yaw_step=%.3f candidates=%d hit=%.3f) "
      "tracking_ndt_leaf(map=%.3f scan=%.3f) sc_threshold=%.3f sc_gap=%.3f "
      "ndt_fitness=%.3f ndt_gap=%.3f early_accept=%d early_fitness=%.3f min_candidates=%d "
      "continuous_localization=%d",
      cloud_topic_.c_str(), odometry_topic_.c_str(), initial_accumulation_sec_,
      scan_context_map_voxel_leaf_size_, scan_context_scan_voxel_leaf_size_,
      initial_ndt_map_voxel_leaf_size_, initial_ndt_scan_voxel_leaf_size_,
      fine_gicp_enabled_ ? 1 : 0, fine_gicp_candidate_count_,
      fine_gicp_map_voxel_leaf_size_, fine_gicp_scan_voxel_leaf_size_,
      fine_gicp_fitness_threshold_, fine_gicp_early_accept_enabled_ ? 1 : 0,
      fine_gicp_early_accept_fitness_, adaptive_fine_gicp_enabled_ ? 1 : 0,
      adaptive_fine_gicp_radius_1_, adaptive_fine_gicp_radius_2_, adaptive_fine_gicp_radius_3_,
      adaptive_fine_gicp_radius_4_, adaptive_fine_gicp_feature_z_min_,
      adaptive_fine_gicp_feature_z_max_, adaptive_fine_gicp_min_source_points_,
      adaptive_fine_gicp_min_target_points_, adaptive_fine_gicp_min_source_xy_cells_,
      adaptive_fine_gicp_min_target_xy_cells_, adaptive_fine_gicp_xy_cell_size_,
      adaptive_fine_gicp_min_scattering_, adaptive_fine_gicp_max_linearity_,
      distance_match_enabled_ ? 1 : 0, distance_match_grid_resolution_,
      distance_match_max_distance_, distance_match_search_xy_radius_,
      distance_match_search_xy_step_, distance_match_search_yaw_range_deg_,
      distance_match_search_yaw_step_deg_, distance_match_candidate_count_,
      distance_match_hit_distance_,
      tracking_ndt_map_voxel_leaf_size_, tracking_ndt_scan_voxel_leaf_size_,
      scan_context_score_threshold_, scan_context_score_gap_threshold_, ndt_fitness_threshold_,
      ndt_fitness_gap_threshold_, initial_ndt_early_accept_enabled_ ? 1 : 0,
      initial_ndt_early_accept_fitness_, initial_ndt_early_accept_min_candidates_,
      continuous_localization_enabled_ ? 1 : 0);
    publish_status("waiting for accumulated scan context input");
  }

private:
  void declare_and_validate_parameters()
  {
    pcd_map_path_ = declare_parameter<std::string>("pcd_map_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "lidar_init");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_odom");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/lio_sam/mapping/cloud_registered");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/lio_sam/mapping/odometry");
    map_preview_topic_ = declare_parameter<std::string>(
      "map_preview_topic", "/localization/map_scan_context_cloud");
    scan_context_map_voxel_leaf_size_ = declare_parameter<double>(
      "scan_context_map_voxel_leaf_size", 0.5);
    scan_context_scan_voxel_leaf_size_ = declare_parameter<double>(
      "scan_context_scan_voxel_leaf_size", 0.5);
    initial_ndt_map_voxel_leaf_size_ = declare_parameter<double>(
      "initial_ndt_map_voxel_leaf_size", 0.5);
    initial_ndt_scan_voxel_leaf_size_ = declare_parameter<double>(
      "initial_ndt_scan_voxel_leaf_size", 0.5);
    fine_gicp_enabled_ = declare_parameter<bool>("fine_gicp_enabled", true);
    fine_gicp_candidate_count_ = declare_parameter<int>("fine_gicp_candidate_count", 2);
    fine_gicp_map_voxel_leaf_size_ = declare_parameter<double>(
      "fine_gicp_map_voxel_leaf_size", 0.15);
    fine_gicp_scan_voxel_leaf_size_ = declare_parameter<double>(
      "fine_gicp_scan_voxel_leaf_size", 0.15);
    distance_match_enabled_ = declare_parameter<bool>("distance_match_enabled", true);
    distance_match_grid_resolution_ = declare_parameter<double>(
      "distance_match_grid_resolution", 0.25);
    distance_match_max_distance_ = declare_parameter<double>("distance_match_max_distance", 3.0);
    distance_match_map_z_min_ = declare_parameter<double>("distance_match_map_z_min", 0.2);
    distance_match_map_z_max_ = declare_parameter<double>("distance_match_map_z_max", 1.5);
    distance_match_scan_z_min_ = declare_parameter<double>("distance_match_scan_z_min", 0.2);
    distance_match_scan_z_max_ = declare_parameter<double>("distance_match_scan_z_max", 1.5);
    distance_match_search_xy_radius_ = declare_parameter<double>(
      "distance_match_search_xy_radius", 2.5);
    distance_match_search_xy_step_ = declare_parameter<double>(
      "distance_match_search_xy_step", 0.25);
    distance_match_search_yaw_range_deg_ = declare_parameter<double>(
      "distance_match_search_yaw_range_deg", 20.0);
    distance_match_search_yaw_step_deg_ = declare_parameter<double>(
      "distance_match_search_yaw_step_deg", 2.0);
    distance_match_candidate_count_ = declare_parameter<int>("distance_match_candidate_count", 5);
    distance_match_hit_distance_ = declare_parameter<double>("distance_match_hit_distance", 0.5);
    distance_match_max_mean_distance_ = declare_parameter<double>(
      "distance_match_max_mean_distance", 1.0);
    distance_match_min_hit_ratio_ = declare_parameter<double>(
      "distance_match_min_hit_ratio", 0.3);
    distance_match_max_far_ratio_ = declare_parameter<double>(
      "distance_match_max_far_ratio", 0.5);
    distance_match_max_out_of_bounds_ratio_ = declare_parameter<double>(
      "distance_match_max_out_of_bounds_ratio", 0.3);
    tracking_ndt_map_voxel_leaf_size_ = declare_parameter<double>(
      "tracking_ndt_map_voxel_leaf_size", 0.2);
    tracking_ndt_scan_voxel_leaf_size_ = declare_parameter<double>(
      "tracking_ndt_scan_voxel_leaf_size", 0.2);
    initial_accumulation_sec_ = declare_parameter<double>("initial_accumulation_sec", 3.0);
    initial_candidate_count_ = declare_parameter<int>("initial_candidate_count", 5);
    scan_context_rings_ = declare_parameter<int>("scan_context_rings", 20);
    scan_context_sectors_ = declare_parameter<int>("scan_context_sectors", 60);
    scan_context_max_radius_ = declare_parameter<double>("scan_context_max_radius", 20.0);
    scan_context_grid_step_ = declare_parameter<double>("scan_context_grid_step", 5.0);
    scan_context_score_threshold_ = declare_parameter<double>("scan_context_score_threshold", 0.65);
    scan_context_score_gap_threshold_ = declare_parameter<double>(
      "scan_context_score_gap_threshold", 0.02);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.5);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 20);
    ndt_fitness_threshold_ = declare_parameter<double>("ndt_fitness_threshold", 2.0);
    ndt_fitness_gap_threshold_ = declare_parameter<double>("ndt_fitness_gap_threshold", 0.02);
    initial_ndt_early_accept_enabled_ = declare_parameter<bool>(
      "initial_ndt_early_accept_enabled", true);
    initial_ndt_early_accept_fitness_ = declare_parameter<double>(
      "initial_ndt_early_accept_fitness", 0.4);
    initial_ndt_early_accept_min_candidates_ = declare_parameter<int>(
      "initial_ndt_early_accept_min_candidates", 2);
    ndt_transformation_epsilon_ = declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.1);
    continuous_localization_enabled_ = declare_parameter<bool>(
      "continuous_localization_enabled", false);
    tracking_failure_limit_ = declare_parameter<int>("tracking_failure_limit", 5);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    initial_ndt_source_radius_ = declare_parameter<double>("initial_ndt_source_radius", 10.0);
    initial_ndt_target_radius_ = declare_parameter<double>("initial_ndt_target_radius", 10.0);
    initial_ndt_z_radius_ = declare_parameter<double>("initial_ndt_z_radius", 5.0);
    fine_gicp_source_radius_ = declare_parameter<double>("fine_gicp_source_radius", 8.0);
    fine_gicp_target_radius_ = declare_parameter<double>("fine_gicp_target_radius", 8.0);
    fine_gicp_z_radius_ = declare_parameter<double>("fine_gicp_z_radius", 5.0);
    fine_gicp_max_correspondence_distance_ = declare_parameter<double>(
      "fine_gicp_max_correspondence_distance", 0.7);
    fine_gicp_max_iterations_ = declare_parameter<int>("fine_gicp_max_iterations", 30);
    fine_gicp_transformation_epsilon_ = declare_parameter<double>(
      "fine_gicp_transformation_epsilon", 0.001);
    fine_gicp_fitness_threshold_ = declare_parameter<double>(
      "fine_gicp_fitness_threshold", 0.5);
    fine_gicp_fitness_gap_threshold_ = declare_parameter<double>(
      "fine_gicp_fitness_gap_threshold", 0.0);
    fine_gicp_early_accept_enabled_ = declare_parameter<bool>(
      "fine_gicp_early_accept_enabled", true);
    fine_gicp_early_accept_fitness_ = declare_parameter<double>(
      "fine_gicp_early_accept_fitness", 0.12);
    adaptive_fine_gicp_enabled_ = declare_parameter<bool>("adaptive_fine_gicp_enabled", true);
    adaptive_fine_gicp_radius_1_ = declare_parameter<double>("adaptive_fine_gicp_radius_1", 4.0);
    adaptive_fine_gicp_radius_2_ = declare_parameter<double>("adaptive_fine_gicp_radius_2", 6.0);
    adaptive_fine_gicp_radius_3_ = declare_parameter<double>("adaptive_fine_gicp_radius_3", 8.0);
    adaptive_fine_gicp_radius_4_ = declare_parameter<double>("adaptive_fine_gicp_radius_4", 10.0);
    adaptive_fine_gicp_feature_z_min_ = declare_parameter<double>(
      "adaptive_fine_gicp_feature_z_min", 0.2);
    adaptive_fine_gicp_feature_z_max_ = declare_parameter<double>(
      "adaptive_fine_gicp_feature_z_max", 1.5);
    adaptive_fine_gicp_min_source_points_ = static_cast<std::size_t>(declare_parameter<int>(
        "adaptive_fine_gicp_min_source_points", 400));
    adaptive_fine_gicp_min_target_points_ = static_cast<std::size_t>(declare_parameter<int>(
        "adaptive_fine_gicp_min_target_points", 1200));
    adaptive_fine_gicp_min_source_xy_cells_ = static_cast<std::size_t>(declare_parameter<int>(
        "adaptive_fine_gicp_min_source_xy_cells", 80));
    adaptive_fine_gicp_min_target_xy_cells_ = static_cast<std::size_t>(declare_parameter<int>(
        "adaptive_fine_gicp_min_target_xy_cells", 120));
    adaptive_fine_gicp_xy_cell_size_ = declare_parameter<double>(
      "adaptive_fine_gicp_xy_cell_size", 0.5);
    adaptive_fine_gicp_min_scattering_ = declare_parameter<double>(
      "adaptive_fine_gicp_min_scattering", 0.01);
    adaptive_fine_gicp_max_linearity_ = declare_parameter<double>(
      "adaptive_fine_gicp_max_linearity", 0.95);
    local_map_radius_ = declare_parameter<double>("local_map_radius", 10.0);
    local_map_z_radius_ = declare_parameter<double>("local_map_z_radius", 5.0);
    initial_pose_z_ = declare_parameter<double>("initial_pose_z", 0.0);
    min_registration_points_ = declare_parameter<int>("min_registration_points", 50);
    min_scan_context_points_ = declare_parameter<int>("min_scan_context_points", 100);
    min_scan_context_bins_ = declare_parameter<int>("min_scan_context_bins", 20);
    initial_max_odometry_translation_ = declare_parameter<double>(
      "initial_max_odometry_translation", 2.0);

    // frame/topic名と探索・NDTの粒度は起動時に検証し、曖昧な失敗を実行中へ持ち込まない。
    if (pcd_map_path_.empty() || map_frame_.empty() || odom_frame_.empty() ||
      lidar_frame_.empty() ||
      cloud_topic_.empty() || odometry_topic_.empty() || map_preview_topic_.empty())
    {
      throw std::invalid_argument("pcd_map_path, frame, and topic parameters must not be empty");
    }
    if (scan_context_map_voxel_leaf_size_ <= 0.0 || scan_context_scan_voxel_leaf_size_ <= 0.0 ||
      initial_ndt_map_voxel_leaf_size_ <= 0.0 || initial_ndt_scan_voxel_leaf_size_ <= 0.0 ||
      fine_gicp_map_voxel_leaf_size_ <= 0.0 || fine_gicp_scan_voxel_leaf_size_ <= 0.0 ||
      tracking_ndt_map_voxel_leaf_size_ <= 0.0 || tracking_ndt_scan_voxel_leaf_size_ <= 0.0 ||
      initial_accumulation_sec_ <= 0.0 || scan_context_max_radius_ <= 0.0 ||
      scan_context_grid_step_ <= 0.0 || ndt_resolution_ <= 0.0 ||
      distance_match_grid_resolution_ <= 0.0 || distance_match_max_distance_ <= 0.0 ||
      distance_match_search_xy_radius_ <= 0.0 || distance_match_search_xy_step_ <= 0.0 ||
      distance_match_search_yaw_range_deg_ < 0.0 || distance_match_search_yaw_step_deg_ <= 0.0 ||
      distance_match_hit_distance_ <= 0.0 ||
      distance_match_max_mean_distance_ <= 0.0 || distance_match_min_hit_ratio_ < 0.0 ||
      distance_match_max_far_ratio_ < 0.0 || distance_match_max_out_of_bounds_ratio_ < 0.0 ||
      initial_ndt_early_accept_fitness_ <= 0.0 ||
      initial_ndt_source_radius_ <= 0.0 || initial_ndt_target_radius_ <= 0.0 ||
      initial_ndt_z_radius_ <= 0.0 || fine_gicp_source_radius_ <= 0.0 ||
      fine_gicp_target_radius_ <= 0.0 || fine_gicp_z_radius_ <= 0.0 ||
      fine_gicp_max_correspondence_distance_ <= 0.0 || fine_gicp_transformation_epsilon_ <= 0.0 ||
      fine_gicp_fitness_threshold_ <= 0.0 || fine_gicp_early_accept_fitness_ <= 0.0 ||
      adaptive_fine_gicp_radius_1_ <= 0.0 || adaptive_fine_gicp_radius_2_ <= 0.0 ||
      adaptive_fine_gicp_radius_3_ <= 0.0 || adaptive_fine_gicp_radius_4_ <= 0.0 ||
      adaptive_fine_gicp_xy_cell_size_ <= 0.0 || adaptive_fine_gicp_min_scattering_ < 0.0 ||
      adaptive_fine_gicp_max_linearity_ < 0.0 ||
      local_map_radius_ <= 0.0 ||
      local_map_z_radius_ <= 0.0)
    {
      throw std::invalid_argument(
              "leaf sizes, radii, accumulation time, and NDT resolution must be positive");
    }
    if (initial_candidate_count_ <= 0 || scan_context_rings_ <= 0 || scan_context_sectors_ <= 0 ||
      ndt_max_iterations_ <= 0 || fine_gicp_candidate_count_ <= 0 ||
      distance_match_candidate_count_ <= 0 ||
      fine_gicp_max_iterations_ <= 0 || initial_ndt_early_accept_min_candidates_ <= 0 ||
      tracking_failure_limit_ <= 0 ||
      min_registration_points_ <= 0 || min_scan_context_points_ <= 0 ||
      min_scan_context_bins_ <= 0)
    {
      throw std::invalid_argument("count parameters must be positive");
    }
    if (distance_match_map_z_min_ >= distance_match_map_z_max_ ||
      distance_match_scan_z_min_ >= distance_match_scan_z_max_ ||
      adaptive_fine_gicp_feature_z_min_ >= adaptive_fine_gicp_feature_z_max_)
    {
      throw std::invalid_argument("distance matching z filter min must be less than max");
    }
    if (!(adaptive_fine_gicp_radius_1_ < adaptive_fine_gicp_radius_2_ &&
      adaptive_fine_gicp_radius_2_ < adaptive_fine_gicp_radius_3_ &&
      adaptive_fine_gicp_radius_3_ < adaptive_fine_gicp_radius_4_))
    {
      throw std::invalid_argument("adaptive fine GICP radii must be strictly increasing");
    }
    if (distance_match_hit_distance_ > distance_match_max_distance_ ||
      distance_match_min_hit_ratio_ > 1.0 || distance_match_max_far_ratio_ > 1.0 ||
      distance_match_max_out_of_bounds_ratio_ > 1.0 ||
      adaptive_fine_gicp_min_scattering_ > 1.0 || adaptive_fine_gicp_max_linearity_ > 1.0)
    {
      throw std::invalid_argument("distance matching thresholds are inconsistent");
    }
  }

  void load_fixed_map()
  {
    pcl::PCLPointCloud2 blob;
    if (pcl::io::loadPCDFile(pcd_map_path_, blob) < 0) {
      throw std::runtime_error("failed to load PCD map: " + pcd_map_path_);
    }

    CloudT::Ptr raw_cloud(new CloudT);
    pcl::fromPCLPointCloud2(blob, *raw_cloud);
    const auto finite_cloud = remove_non_finite_points(raw_cloud);
    scan_context_map_cloud_ = voxel_downsample(finite_cloud, scan_context_map_voxel_leaf_size_);
    initial_ndt_map_cloud_ = voxel_downsample(finite_cloud, initial_ndt_map_voxel_leaf_size_);
    fine_gicp_map_cloud_ = voxel_downsample(finite_cloud, fine_gicp_map_voxel_leaf_size_);
    tracking_ndt_map_cloud_ = voxel_downsample(finite_cloud, tracking_ndt_map_voxel_leaf_size_);

    if (scan_context_map_cloud_->empty() || initial_ndt_map_cloud_->empty() ||
      fine_gicp_map_cloud_->empty() || tracking_ndt_map_cloud_->empty())
    {
      throw std::runtime_error(
              "PCD map has no finite points after downsampling: " + pcd_map_path_);
    }
    if (distance_match_enabled_) {
      build_distance_field_map(finite_cloud);
    }
    RCLCPP_INFO(
      get_logger(),
      "Loaded PCD map: path=%s raw=%zu finite=%zu sc_points=%zu initial_ndt_points=%zu "
      "fine_gicp_points=%zu tracking_ndt_points=%zu sc_leaf=%.3f initial_ndt_leaf=%.3f "
      "fine_gicp_leaf=%.3f tracking_ndt_leaf=%.3f",
      pcd_map_path_.c_str(), raw_cloud->size(), finite_cloud->size(),
      scan_context_map_cloud_->size(), initial_ndt_map_cloud_->size(),
      fine_gicp_map_cloud_->size(), tracking_ndt_map_cloud_->size(),
      scan_context_map_voxel_leaf_size_, initial_ndt_map_voxel_leaf_size_,
      fine_gicp_map_voxel_leaf_size_, tracking_ndt_map_voxel_leaf_size_);
  }

  void build_distance_field_map(const CloudT::ConstPtr & map_cloud)
  {
    const auto build_start = std::chrono::steady_clock::now();
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    std::size_t filtered_points = 0;

    // 床面や天井の投影でoccupancyが埋まらないよう、壁・障害物に相当する高さだけを使う。
    for (const auto & point : map_cloud->points) {
      if (!is_finite_point(point) || point.z < distance_match_map_z_min_ ||
        point.z > distance_match_map_z_max_)
      {
        continue;
      }
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
      ++filtered_points;
    }

    if (filtered_points == 0U) {
      throw std::runtime_error("distance matching map has no points after z filtering");
    }

    const double padding = distance_match_max_distance_ + distance_match_search_xy_radius_ +
      distance_match_grid_resolution_;
    distance_field_map_.resolution = distance_match_grid_resolution_;
    distance_field_map_.origin_x = static_cast<double>(min_x) - padding;
    distance_field_map_.origin_y = static_cast<double>(min_y) - padding;
    distance_field_map_.width = static_cast<int>(
      std::ceil(
        (static_cast<double>(max_x) - static_cast<double>(min_x) + 2.0 * padding) /
        distance_match_grid_resolution_)) + 1;
    distance_field_map_.height = static_cast<int>(
      std::ceil(
        (static_cast<double>(max_y) - static_cast<double>(min_y) + 2.0 * padding) /
        distance_match_grid_resolution_)) + 1;
    if (distance_field_map_.width <= 0 || distance_field_map_.height <= 0) {
      throw std::runtime_error("distance matching map grid dimensions are invalid");
    }

    cv::Mat occupancy(
      distance_field_map_.height, distance_field_map_.width, CV_8UC1, cv::Scalar(255));
    std::size_t occupied_cells = 0;
    for (const auto & point : map_cloud->points) {
      if (!is_finite_point(point) || point.z < distance_match_map_z_min_ ||
        point.z > distance_match_map_z_max_)
      {
        continue;
      }
      const auto cell = distance_map_cell(point.x, point.y);
      if (!is_distance_cell_inside(cell.first, cell.second)) {
        continue;
      }
      auto & value = occupancy.at<unsigned char>(cell.second, cell.first);
      if (value != 0U) {
        value = 0U;
        ++occupied_cells;
      }
    }

    cv::Mat distance_pixels;
    cv::distanceTransform(occupancy, distance_pixels, cv::DIST_L2, cv::DIST_MASK_5);
    distance_pixels.convertTo(
      distance_field_map_.distance_meters, CV_32FC1, distance_match_grid_resolution_);
    distance_field_map_.occupied_cells = occupied_cells;
    distance_field_map_.ready = true;

    RCLCPP_INFO(
      get_logger(),
      "Built distance transform map: size=%dx%d resolution=%.3f origin=(%.3f, %.3f) "
      "filtered_points=%zu occupied_cells=%zu z_range=(%.3f, %.3f) duration_ms=%.3f",
      distance_field_map_.width, distance_field_map_.height, distance_field_map_.resolution,
      distance_field_map_.origin_x, distance_field_map_.origin_y, filtered_points, occupied_cells,
      distance_match_map_z_min_, distance_match_map_z_max_,
      elapsed_milliseconds(build_start, std::chrono::steady_clock::now()));
  }

  std::pair<int, int> distance_map_cell(const double x, const double y) const
  {
    const int col = static_cast<int>(
      std::floor((x - distance_field_map_.origin_x) / distance_field_map_.resolution));
    const int row = static_cast<int>(
      std::floor((y - distance_field_map_.origin_y) / distance_field_map_.resolution));
    return {col, row};
  }

  bool is_distance_cell_inside(const int col, const int row) const
  {
    return col >= 0 && row >= 0 && col < distance_field_map_.width &&
           row < distance_field_map_.height;
  }

  void build_map_context_index()
  {
    const auto bounds = compute_bounds(scan_context_map_cloud_);
    for (double x = bounds.min_x; x <= bounds.max_x; x += scan_context_grid_step_) {
      for (double y = bounds.min_y; y <= bounds.max_y; y += scan_context_grid_step_) {
        Eigen::Vector3f center(
          static_cast<float>(x), static_cast<float>(y), static_cast<float>(initial_pose_z_));
        auto submap = extract_cloud_neighborhood(
          scan_context_map_cloud_, center, scan_context_max_radius_, local_map_z_radius_);
        if (submap->size() < static_cast<std::size_t>(min_scan_context_points_)) {
          continue;
        }

        // grid中心ごとの粗いdescriptorだけを保持し、NDTでは別密度のmapを再抽出する。
        auto descriptor = make_scan_context_descriptor(
          submap, center, scan_context_rings_, scan_context_sectors_, scan_context_max_radius_);
        if (descriptor.occupied_count < static_cast<std::size_t>(min_scan_context_bins_)) {
          continue;
        }
        map_contexts_.push_back(MapContext{center, std::move(submap), std::move(descriptor)});
      }
    }

    if (map_contexts_.empty()) {
      throw std::runtime_error("failed to build Scan Context index from PCD map");
    }
    RCLCPP_INFO(
      get_logger(),
      "Built Scan Context index: contexts=%zu rings=%d sectors=%d radius=%.2f step=%.2f",
      map_contexts_.size(), scan_context_rings_, scan_context_sectors_, scan_context_max_radius_,
      scan_context_grid_step_);
  }

  void publish_map_preview()
  {
    PointCloud2 message;
    pcl::toROSMsg(*scan_context_map_cloud_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();

    // 固定地図の粗い点群は静的表示用なので、transient local publisherへ一度だけ載せる。
    map_preview_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(),
      "Published PCD localization map preview: topic=%s frame=%s points=%zu leaf=%.3f",
      map_preview_topic_.c_str(), map_frame_.c_str(), scan_context_map_cloud_->size(),
      scan_context_map_voxel_leaf_size_);
  }

  void handle_odometry(const Odometry::SharedPtr message)
  {
    if (message == nullptr) {
      return;
    }

    if (!message->header.frame_id.empty() && message->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Odometry parent frame is %s, expected %s.",
        message->header.frame_id.c_str(), odom_frame_.c_str());
    }
    if (!message->child_frame_id.empty() && message->child_frame_id != lidar_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Odometry child frame is %s, expected %s.",
        message->child_frame_id.c_str(), lidar_frame_.c_str());
    }

    // LIO-SAM odometryはlidar_init->lidar_odomとして扱い、
    // 固定mapとの結合はlocalizerだけが担当する。
    const rclcpp::Time stamp(message->header.stamp);
    const auto lidar_init_from_lidar_odom = pose_to_transform(message->pose.pose);
    latest_odometry_ = StampedOdometry{stamp, lidar_init_from_lidar_odom};
    ++odometry_count_;
    if (odometry_count_ == 1U) {
      const auto & origin = lidar_init_from_lidar_odom.getOrigin();
      RCLCPP_INFO(
        get_logger(),
        "First LIO-SAM odometry received: stamp=%.6f frame=%s child=%s "
        "translation=(%.3f, %.3f, %.3f)",
        stamp.seconds(), message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        origin.x(), origin.y(), origin.z());
    }
  }

  void handle_cloud(const PointCloud2::SharedPtr message)
  {
    if (message == nullptr) {
      return;
    }
    ++cloud_message_count_;
    const auto raw_point_count = static_cast<std::size_t>(message->width) *
      static_cast<std::size_t>(message->height);
    const auto stamp = cloud_stamp(*message);
    latest_cloud_stamp_ = stamp;
    if (cloud_message_count_ == 1U) {
      RCLCPP_INFO(
        get_logger(),
        "First localization cloud message received: stamp=%.6f frame=%s raw_points=%zu",
        stamp.seconds(), message->header.frame_id.c_str(), raw_point_count);
    }
    if (!latest_odometry_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for LIO-SAM odometry before localization.");
      return;
    }

    if (state_ == LocalizationState::TRACKING && !continuous_localization_enabled_) {
      // 初期位置確定後はPCD照合を止め、固定したmap->lidar_initだけを継続配信する。
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Continuous PCD localization is disabled; republish fixed map -> lidar_init.");
      publish_pose_and_tf(stamp);
      return;
    }

    const double scan_leaf_size = (state_ == LocalizationState::TRACKING) ?
      tracking_ndt_scan_voxel_leaf_size_ : initial_ndt_scan_voxel_leaf_size_;
    auto scan_cloud = cloud_from_message(*message, scan_leaf_size);
    ++processed_cloud_count_;
    if (processed_cloud_count_ == 1U) {
      RCLCPP_INFO(
        get_logger(),
        "First localization cloud processed: stamp=%.6f frame=%s raw_points=%zu "
        "downsampled_points=%zu leaf=%.3f",
        stamp.seconds(), message->header.frame_id.c_str(), raw_point_count, scan_cloud->size(),
        scan_leaf_size);
    }
    if (scan_cloud->size() < static_cast<std::size_t>(min_registration_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Drop scan because finite points are too few: %zu",
        scan_cloud->size());
      return;
    }

    const auto odometry = *latest_odometry_;
    if (state_ == LocalizationState::TRACKING) {
      track_scan(scan_cloud, odometry, stamp);
      return;
    }

    if (state_ == LocalizationState::LOST && !continuous_localization_enabled_) {
      // 継続補正を使わない検証ではLOST後の再localizationも行わず、SLAM側の挙動確認を優先する。
      publish_status("continuous PCD relocalization is disabled");
      return;
    }

    if (state_ == LocalizationState::LOST) {
      transition_to(
        LocalizationState::RELOCALIZING,
        "start global relocalization after tracking loss");
    }
    accumulate_initial_scan(scan_cloud, odometry, stamp);
    if (is_accumulation_ready(stamp)) {
      try_global_initialization(stamp);
    }
  }

  CloudT::Ptr cloud_from_message(const PointCloud2 & message, const double leaf_size) const
  {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(message, *cloud);
    auto finite_cloud = remove_non_finite_points(cloud);
    return voxel_downsample(finite_cloud, leaf_size);
  }

  rclcpp::Time cloud_stamp(const PointCloud2 & message) const
  {
    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() == 0) {
      return now();
    }
    return stamp;
  }

  void reset_accumulation()
  {
    accumulated_scan_lidar_init_.reset(new CloudT);
    accumulation_start_stamp_.reset();
    accumulation_start_odometry_.reset();
  }

  void accumulate_initial_scan(
    const CloudT::ConstPtr & scan_cloud, const StampedOdometry & odometry,
    const rclcpp::Time & stamp)
  {
    if (!accumulation_start_stamp_.has_value()) {
      reset_accumulation();
      accumulation_start_stamp_ = stamp;
      accumulation_start_odometry_ = odometry.lidar_init_from_lidar_odom;
      RCLCPP_INFO(
        get_logger(), "Start localization initial accumulation: stamp=%.6f required_duration=%.3f",
        stamp.seconds(), initial_accumulation_sec_);
    }

    if (initial_max_odometry_translation_ > 0.0 && accumulation_start_odometry_.has_value()) {
      const auto delta = accumulation_start_odometry_->inverse() *
        odometry.lidar_init_from_lidar_odom;
      if (delta.getOrigin().length() > initial_max_odometry_translation_) {
        RCLCPP_WARN(
          get_logger(),
          "Reset initial accumulation because odometry moved too far: distance=%.3f limit=%.3f",
          delta.getOrigin().length(), initial_max_odometry_translation_);
        reset_accumulation();
        accumulation_start_stamp_ = stamp;
        accumulation_start_odometry_ = odometry.lidar_init_from_lidar_odom;
      }
    }

    // 初期化用scanはLIO-SAM odometryでlidar_init frameへ合成し、
    // NDT結果をmap->lidar_initとして得る。
    CloudT transformed_scan;
    pcl::transformPointCloud(
      *scan_cloud, transformed_scan, transform_to_eigen(odometry.lidar_init_from_lidar_odom));
    accumulated_scan_lidar_init_->insert(
      accumulated_scan_lidar_init_->end(), transformed_scan.begin(), transformed_scan.end());
    accumulated_scan_lidar_init_->width =
      static_cast<std::uint32_t>(accumulated_scan_lidar_init_->size());
    accumulated_scan_lidar_init_->height = 1;
    accumulated_scan_lidar_init_->is_dense = true;
    if (accumulation_start_stamp_.has_value()) {
      const double duration = (stamp - *accumulation_start_stamp_).seconds();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Initial accumulation progress: duration=%.3f/%.3f points=%zu latest_scan_points=%zu",
        duration, initial_accumulation_sec_, accumulated_scan_lidar_init_->size(),
        scan_cloud->size());
    }
  }

  bool is_accumulation_ready(const rclcpp::Time & stamp) const
  {
    if (!accumulation_start_stamp_.has_value()) {
      return false;
    }
    return (stamp - *accumulation_start_stamp_).seconds() >= initial_accumulation_sec_ &&
           accumulated_scan_lidar_init_->size() >=
           static_cast<std::size_t>(min_registration_points_);
  }

  std::vector<Eigen::Vector2f> make_distance_scan_cells(const CloudT::ConstPtr & scan_cloud) const
  {
    std::vector<Eigen::Vector2f> cells;
    std::unordered_set<std::uint64_t> seen_cells;
    cells.reserve(scan_cloud->size());
    seen_cells.reserve(scan_cloud->size());

    // 非反復スキャンの点密度差を抑えるため、distance matchingではscan側もgrid cell単位に圧縮する。
    for (const auto & point : scan_cloud->points) {
      if (!is_finite_point(point) || point.z < distance_match_scan_z_min_ ||
        point.z > distance_match_scan_z_max_)
      {
        continue;
      }
      const int cell_x = static_cast<int>(std::floor(point.x / distance_match_grid_resolution_));
      const int cell_y = static_cast<int>(std::floor(point.y / distance_match_grid_resolution_));
      if (!seen_cells.insert(make_cell_key(cell_x, cell_y)).second) {
        continue;
      }
      cells.emplace_back(
        static_cast<float>((static_cast<double>(cell_x) + 0.5) * distance_match_grid_resolution_),
        static_cast<float>((static_cast<double>(cell_y) + 0.5) * distance_match_grid_resolution_));
    }
    return cells;
  }

  DistanceScore score_distance_pose(
    const std::vector<Eigen::Vector2f> & scan_cells, const tf2::Transform & map_from_lidar_init)
  const
  {
    DistanceScore result;
    if (!distance_field_map_.ready || scan_cells.empty()) {
      return result;
    }

    const double yaw = yaw_from_transform(map_from_lidar_init);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const auto & origin = map_from_lidar_init.getOrigin();
    std::size_t in_bounds_count = 0;
    std::size_t hit_count = 0;
    std::size_t far_count = 0;
    std::size_t out_of_bounds_count = 0;
    double distance_sum = 0.0;

    // 各scan cellを候補poseでmap gridへ投影し、最寄りmap occupiedまでの距離で整合性を評価する。
    for (const auto & cell : scan_cells) {
      const double map_x = cos_yaw * cell.x() - sin_yaw * cell.y() + origin.x();
      const double map_y = sin_yaw * cell.x() + cos_yaw * cell.y() + origin.y();
      const auto map_cell = distance_map_cell(map_x, map_y);
      if (!is_distance_cell_inside(map_cell.first, map_cell.second)) {
        ++out_of_bounds_count;
        continue;
      }

      const double distance = std::min(
        static_cast<double>(
          distance_field_map_.distance_meters.at<float>(map_cell.second, map_cell.first)),
        distance_match_max_distance_);
      distance_sum += distance;
      ++in_bounds_count;
      if (distance <= distance_match_hit_distance_) {
        ++hit_count;
      }
      if (distance >= distance_match_max_distance_) {
        ++far_count;
      }
    }

    if (in_bounds_count == 0U) {
      return result;
    }
    const double total_count = static_cast<double>(scan_cells.size());
    result.valid = true;
    result.evaluated_count = in_bounds_count;
    result.mean_distance = distance_sum / static_cast<double>(in_bounds_count);
    result.hit_ratio = static_cast<double>(hit_count) / total_count;
    result.far_ratio = static_cast<double>(far_count) / total_count;
    result.out_of_bounds_ratio = static_cast<double>(out_of_bounds_count) / total_count;
    result.score = result.mean_distance + result.far_ratio + 2.0 * result.out_of_bounds_ratio -
      0.5 * result.hit_ratio;
    return result;
  }

  bool is_distance_score_accepted(const DistanceScore & score) const
  {
    return score.valid && score.mean_distance <= distance_match_max_mean_distance_ &&
           score.hit_ratio >= distance_match_min_hit_ratio_ &&
           score.far_ratio <= distance_match_max_far_ratio_ &&
           score.out_of_bounds_ratio <= distance_match_max_out_of_bounds_ratio_;
  }

  bool is_initial_result_better(const InitialResult & lhs, const InitialResult & rhs) const
  {
    // DTで絞り込めた候補は形状一致を主順位にし、registration fitnessは同点時だけ使う。
    if (lhs.refined_with_distance && rhs.refined_with_distance) {
      if (lhs.distance_score.score != rhs.distance_score.score) {
        return lhs.distance_score.score < rhs.distance_score.score;
      }
      return lhs.registration.fitness < rhs.registration.fitness;
    }
    if (lhs.refined_with_distance != rhs.refined_with_distance) {
      return lhs.refined_with_distance;
    }
    return lhs.registration.fitness < rhs.registration.fitness;
  }

  bool should_early_accept_ndt(
    const std::size_t accepted_candidate_count, const InitialGuessCandidate & guess_candidate,
    const RegistrationResult & registration) const
  {
    if (!initial_ndt_early_accept_enabled_ || !guess_candidate.refined_with_distance) {
      return false;
    }
    if (accepted_candidate_count <
      static_cast<std::size_t>(initial_ndt_early_accept_min_candidates_))
    {
      return false;
    }
    return registration.converged &&
           registration.fitness <= initial_ndt_early_accept_fitness_;
  }

  bool should_early_accept_gicp(const RegistrationResult & registration) const
  {
    return fine_gicp_early_accept_enabled_ && registration.converged &&
           registration.fitness <= fine_gicp_early_accept_fitness_;
  }

  std::vector<InitialGuessCandidate> make_fallback_initial_guesses(
    const std::vector<CandidateMatch> & candidates) const
  {
    std::vector<InitialGuessCandidate> guesses;
    guesses.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      const auto & center = candidate.context->center;
      guesses.push_back(
        InitialGuessCandidate{
          candidate, make_planar_transform(center.x(), center.y(), initial_pose_z_, candidate.yaw),
          DistanceScore{}, false});
    }
    return guesses;
  }

  std::vector<InitialGuessCandidate> refine_initial_candidates_with_distance_transform(
    const std::vector<CandidateMatch> & candidates,
    const std::vector<Eigen::Vector2f> & scan_cells) const
  {
    std::vector<InitialGuessCandidate> refined_guesses;
    if (!distance_match_enabled_ || !distance_field_map_.ready) {
      return refined_guesses;
    }
    if (scan_cells.size() < static_cast<std::size_t>(min_registration_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Skip distance matching because scan_cells=%zu is below min_registration_points=%d",
        scan_cells.size(), min_registration_points_);
      return refined_guesses;
    }

    const int xy_steps = static_cast<int>(
      std::floor(distance_match_search_xy_radius_ / distance_match_search_xy_step_));
    const double yaw_range = distance_match_search_yaw_range_deg_ * kPi / 180.0;
    const double yaw_step = distance_match_search_yaw_step_deg_ * kPi / 180.0;
    const int yaw_steps = static_cast<int>(std::floor(yaw_range / yaw_step));

    for (std::size_t index = 0; index < candidates.size(); ++index) {
      const auto & candidate = candidates[index];
      const auto & center = candidate.context->center;
      DistanceScore best_score;
      auto best_transform = make_planar_transform(
        center.x(), center.y(), initial_pose_z_, candidate.yaw);

      // Scan Context候補の周辺だけを格子探索し、壁・障害物距離場に最も近いposeを選ぶ。
      for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
        const double dx = static_cast<double>(ix) * distance_match_search_xy_step_;
        for (int iy = -xy_steps; iy <= xy_steps; ++iy) {
          const double dy = static_cast<double>(iy) * distance_match_search_xy_step_;
          for (int iyaw = -yaw_steps; iyaw <= yaw_steps; ++iyaw) {
            const double yaw =
              normalize_angle(candidate.yaw + static_cast<double>(iyaw) * yaw_step);
            const auto guess = make_planar_transform(
              static_cast<double>(center.x()) + dx, static_cast<double>(center.y()) + dy,
              initial_pose_z_, yaw);
            const auto score = score_distance_pose(scan_cells, guess);
            if (score.valid && score.score < best_score.score) {
              best_score = score;
              best_transform = guess;
            }
          }
        }
      }

      const bool accepted = is_distance_score_accepted(best_score);
      const auto & best_origin = best_transform.getOrigin();
      RCLCPP_INFO(
        get_logger(),
        "Distance match candidate: attempt=%zu index=%zu/%zu base_center=(%.3f, %.3f, %.3f) "
        "base_yaw=%.6f best_pose=(%.3f, %.3f, %.3f, %.6f) scan_cells=%zu "
        "score=%.6f mean=%.6f hit=%.6f far=%.6f out=%.6f accepted=%d",
        initialization_attempt_count_, index + 1U, candidates.size(), center.x(), center.y(),
        center.z(), candidate.yaw, best_origin.x(), best_origin.y(), best_origin.z(),
        yaw_from_transform(best_transform), scan_cells.size(), best_score.score,
        best_score.mean_distance, best_score.hit_ratio, best_score.far_ratio,
        best_score.out_of_bounds_ratio, accepted ? 1 : 0);
      if (accepted) {
        refined_guesses.push_back(
          InitialGuessCandidate{candidate, best_transform, best_score, true});
      }
    }

    std::sort(
      refined_guesses.begin(), refined_guesses.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.distance_score.score < rhs.distance_score.score;
      });
    if (refined_guesses.size() > static_cast<std::size_t>(distance_match_candidate_count_)) {
      refined_guesses.resize(static_cast<std::size_t>(distance_match_candidate_count_));
    }
    return refined_guesses;
  }

  std::vector<CandidateMatch> find_initial_candidates(
    const ScanContextDescriptor & query_descriptor) const
  {
    std::vector<CandidateMatch> candidates;
    candidates.reserve(map_contexts_.size());
    for (const auto & context : map_contexts_) {
      auto [score, shift] = best_descriptor_alignment(query_descriptor, context.descriptor);
      if (!std::isfinite(score)) {
        continue;
      }

      // sector shiftを概略yawへ変換し、PCL NDTには候補submap中心とyawだけを初期値として渡す。
      const double yaw = static_cast<double>(shift) * kTwoPi /
        static_cast<double>(scan_context_sectors_);
      candidates.push_back(CandidateMatch{&context, score, shift, yaw});
    }

    std::sort(
      candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.score < rhs.score;
      });
    if (candidates.size() > static_cast<std::size_t>(initial_candidate_count_)) {
      candidates.resize(static_cast<std::size_t>(initial_candidate_count_));
    }
    return candidates;
  }

  void try_global_initialization(const rclcpp::Time & stamp)
  {
    const auto initialization_start = std::chrono::steady_clock::now();
    ++initialization_attempt_count_;
    auto scan_context_source = voxel_downsample(
      accumulated_scan_lidar_init_, scan_context_scan_voxel_leaf_size_);
    auto ndt_source_full = voxel_downsample(
      accumulated_scan_lidar_init_, initial_ndt_scan_voxel_leaf_size_);
    auto ndt_source = extract_cloud_neighborhood(
      ndt_source_full, Eigen::Vector3f::Zero(), initial_ndt_source_radius_, initial_ndt_z_radius_);
    auto fine_gicp_source_full = voxel_downsample(
      accumulated_scan_lidar_init_, fine_gicp_scan_voxel_leaf_size_);
    RCLCPP_INFO(
      get_logger(),
      "Initialization attempt %zu: stamp=%.6f accumulated_points=%zu sc_source_points=%zu "
      "ndt_source_points=%zu ndt_source_full_points=%zu fine_gicp_source_full_points=%zu",
      initialization_attempt_count_, stamp.seconds(), accumulated_scan_lidar_init_->size(),
      scan_context_source->size(), ndt_source->size(), ndt_source_full->size(),
      fine_gicp_source_full->size());
    if (ndt_source->size() < static_cast<std::size_t>(min_registration_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: ndt_source_points=%zu min_registration_points=%d",
        initialization_attempt_count_, ndt_source->size(), min_registration_points_);
      publish_status("accumulated scan is too small for initialization");
      return;
    }

    const auto query_descriptor = make_scan_context_descriptor(
      scan_context_source, Eigen::Vector3f::Zero(), scan_context_rings_, scan_context_sectors_,
      scan_context_max_radius_);
    if (query_descriptor.occupied_count < static_cast<std::size_t>(min_scan_context_bins_)) {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: scan_context_bins=%zu min_scan_context_bins=%d",
        initialization_attempt_count_, query_descriptor.occupied_count, min_scan_context_bins_);
      publish_status("accumulated scan has too few Scan Context bins");
      return;
    }

    const auto candidates = find_initial_candidates(query_descriptor);
    const double best_score = candidates.empty() ? std::numeric_limits<double>::infinity() :
      candidates.front().score;
    const double second_score = candidates.size() > 1U ? candidates[1].score :
      std::numeric_limits<double>::infinity();
    const double score_gap = candidates.size() > 1U ? second_score - best_score :
      std::numeric_limits<double>::infinity();
    RCLCPP_INFO(
      get_logger(),
      "Initialization Scan Context: attempt=%zu occupied_bins=%zu candidates=%zu "
      "best=%.6f second=%.6f gap=%.6f threshold=%.6f gap_threshold=%.6f",
      initialization_attempt_count_, query_descriptor.occupied_count, candidates.size(), best_score,
      second_score, score_gap, scan_context_score_threshold_, scan_context_score_gap_threshold_);
    if (candidates.empty()) {
      RCLCPP_WARN(
        get_logger(), "Initialization attempt %zu rejected: no Scan Context candidates",
        initialization_attempt_count_);
      publish_status("no Scan Context candidate for initialization");
      return;
    }
    if (candidates.front().score > scan_context_score_threshold_) {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: best Scan Context score %.6f exceeds threshold %.6f",
        initialization_attempt_count_, candidates.front().score, scan_context_score_threshold_);
      publish_status("best Scan Context candidate exceeds score threshold");
      compact_accumulated_scan();
      return;
    }
    if (candidates.size() > 1U && scan_context_score_gap_threshold_ > 0.0 &&
      candidates[1].score - candidates[0].score < scan_context_score_gap_threshold_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: Scan Context gap %.6f below threshold %.6f",
        initialization_attempt_count_, candidates[1].score - candidates[0].score,
        scan_context_score_gap_threshold_);
      publish_status("Scan Context candidates are ambiguous");
      compact_accumulated_scan();
      return;
    }

    auto initial_guesses = make_fallback_initial_guesses(candidates);
    if (distance_match_enabled_) {
      const auto distance_scan_cells = make_distance_scan_cells(accumulated_scan_lidar_init_);
      RCLCPP_INFO(
        get_logger(),
        "Initialization distance matching input: attempt=%zu scan_cells=%zu z_range=(%.3f, %.3f)",
        initialization_attempt_count_, distance_scan_cells.size(), distance_match_scan_z_min_,
        distance_match_scan_z_max_);
      auto distance_guesses = refine_initial_candidates_with_distance_transform(
        candidates, distance_scan_cells);
      if (!distance_guesses.empty()) {
        initial_guesses = std::move(distance_guesses);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Initialization attempt %zu: distance matching accepted no candidates; fallback to "
          "Scan Context guesses",
          initialization_attempt_count_);
      }
    }

    std::vector<InitialResult> accepted_results;
    for (std::size_t index = 0; index < initial_guesses.size(); ++index) {
      const auto & guess_candidate = initial_guesses[index];
      const auto & candidate = guess_candidate.candidate;
      const auto & guess = guess_candidate.map_from_lidar_init;
      const auto origin = guess.getOrigin();
      auto ndt_target = extract_cloud_neighborhood(
        initial_ndt_map_cloud_, Eigen::Vector3f(
          origin.x(), origin.y(),
          origin.z()), initial_ndt_target_radius_, initial_ndt_z_radius_);
      if (ndt_target->size() < static_cast<std::size_t>(min_registration_points_)) {
        RCLCPP_WARN(
          get_logger(),
          "Initialization NDT candidate skipped: attempt=%zu index=%zu/%zu target_points=%zu "
          "min_registration_points=%d guess=(%.3f, %.3f, %.3f, %.6f)",
          initialization_attempt_count_, index + 1U, initial_guesses.size(), ndt_target->size(),
          min_registration_points_, origin.x(), origin.y(), origin.z(), yaw_from_transform(guess));
        continue;
      }
      const auto ndt_start = std::chrono::steady_clock::now();
      auto registration = run_ndt(ndt_source, ndt_target, guess);
      const double ndt_ms = elapsed_milliseconds(ndt_start, std::chrono::steady_clock::now());
      const auto registered_origin = registration.target_from_source.getOrigin();
      RCLCPP_INFO(
        get_logger(),
        "Initialization NDT candidate: attempt=%zu index=%zu/%zu guess=(%.3f, %.3f, %.3f, %.6f) "
        "result=(%.3f, %.3f, %.3f, %.6f) source_points=%zu target_points=%zu sc_score=%.6f "
        "base_yaw=%.6f distance_refined=%d distance_score=%.6f distance_mean=%.6f "
        "duration_ms=%.3f converged=%d fitness=%.6f accepted=%d",
        initialization_attempt_count_, index + 1U, initial_guesses.size(), origin.x(), origin.y(),
        origin.z(), yaw_from_transform(guess), registered_origin.x(), registered_origin.y(),
        registered_origin.z(), yaw_from_transform(registration.target_from_source),
        ndt_source->size(), ndt_target->size(), candidate.score, candidate.yaw,
        guess_candidate.refined_with_distance ? 1 : 0, guess_candidate.distance_score.score,
        guess_candidate.distance_score.mean_distance, ndt_ms, registration.converged ? 1 : 0,
        registration.fitness,
        (registration.converged && registration.fitness <= ndt_fitness_threshold_) ? 1 : 0);
      if (registration.converged && registration.fitness <= ndt_fitness_threshold_) {
        accepted_results.push_back(
          InitialResult{candidate, registration, registration.fitness, false,
            guess_candidate.distance_score, guess_candidate.refined_with_distance});

        // DTで絞り込んだ候補が十分低fitnessなら、最低件数確認後に残りNDTを省略して初期化を短縮する。
        if (should_early_accept_ndt(accepted_results.size(), guess_candidate, registration)) {
          RCLCPP_INFO(
            get_logger(),
            "Initialization NDT early accepted: attempt=%zu accepted_candidates=%zu "
            "fitness=%.6f threshold=%.6f min_candidates=%d",
            initialization_attempt_count_, accepted_results.size(), registration.fitness,
            initial_ndt_early_accept_fitness_, initial_ndt_early_accept_min_candidates_);
          break;
        }
      }
    }

    if (accepted_results.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: NDT accepted no candidates fitness_threshold=%.6f",
        initialization_attempt_count_, ndt_fitness_threshold_);
      publish_status("NDT rejected all Scan Context initialization candidates");
      compact_accumulated_scan();
      return;
    }
    std::sort(
      accepted_results.begin(), accepted_results.end(), [this](const auto & lhs, const auto & rhs) {
        return is_initial_result_better(lhs, rhs);
      });
    auto final_results = accepted_results;
    if (fine_gicp_enabled_) {
      final_results = refine_initial_candidates_with_gicp(accepted_results, fine_gicp_source_full);
      if (final_results.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Initialization attempt %zu: fine GICP accepted no candidates; fallback to coarse NDT",
          initialization_attempt_count_);
        final_results = accepted_results;
      }
    }

    // DT主順位ではregistration fitness差を曖昧判定に使わず、fallback時だけ従来基準を適用する。
    const double final_gap_threshold = final_results.front().refined_with_distance ? 0.0 :
      (final_results.front().refined_with_gicp ? fine_gicp_fitness_gap_threshold_ :
      ndt_fitness_gap_threshold_);
    if (final_results.size() > 1U && final_gap_threshold > 0.0 &&
      final_results[1].registration.fitness - final_results[0].registration.fitness <
      final_gap_threshold)
    {
      RCLCPP_WARN(
        get_logger(),
        "Initialization attempt %zu rejected: final registration fitness gap %.6f below "
        "threshold %.6f best=%.6f second=%.6f refined_with_gicp=%d",
        initialization_attempt_count_,
        final_results[1].registration.fitness - final_results[0].registration.fitness,
        final_gap_threshold, final_results[0].registration.fitness,
        final_results[1].registration.fitness, final_results.front().refined_with_gicp ? 1 : 0);
      publish_status("initialization candidates are ambiguous after registration");
      compact_accumulated_scan();
      return;
    }

    last_map_to_lidar_init_ = final_results.front().registration.target_from_source;
    const auto final_origin = last_map_to_lidar_init_.getOrigin();
    RCLCPP_INFO(
      get_logger(),
      "Initialization final map->lidar_init: translation=(%.3f, %.3f, %.3f) yaw=%.6f "
      "fitness=%.6f refined_with_distance=%d refined_with_gicp=%d",
      final_origin.x(), final_origin.y(), final_origin.z(),
      yaw_from_transform(last_map_to_lidar_init_),
      final_results.front().registration.fitness,
      final_results.front().refined_with_distance ? 1 : 0,
      final_results.front().refined_with_gicp ? 1 : 0);
    tracking_failure_count_ = 0;
    reset_accumulation();
    transition_to(LocalizationState::TRACKING, initialization_summary(final_results.front()));
    RCLCPP_INFO(
      get_logger(), "Initialization attempt %zu accepted: total_duration_ms=%.3f",
      initialization_attempt_count_,
      elapsed_milliseconds(initialization_start, std::chrono::steady_clock::now()));
    publish_pose_and_tf(stamp);
  }

  std::vector<InitialResult> refine_initial_candidates_with_gicp(
    const std::vector<InitialResult> & coarse_results, const CloudT::ConstPtr & source_cloud)
  {
    std::vector<InitialResult> refined_results;
    if (source_cloud->size() < static_cast<std::size_t>(min_registration_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Skip fine GICP because source_full_points=%zu is below min_registration_points=%d",
        source_cloud->size(), min_registration_points_);
      return refined_results;
    }

    const std::vector<double> adaptive_radii{
      adaptive_fine_gicp_radius_1_, adaptive_fine_gicp_radius_2_, adaptive_fine_gicp_radius_3_,
      adaptive_fine_gicp_radius_4_};
    const auto refine_count = std::min<std::size_t>(
      coarse_results.size(), static_cast<std::size_t>(fine_gicp_candidate_count_));
    refined_results.reserve(refine_count);
    for (std::size_t index = 0; index < refine_count; ++index) {
      const auto & coarse_result = coarse_results[index];
      const auto origin = coarse_result.registration.target_from_source.getOrigin();
      CloudT::ConstPtr selected_source_cloud;
      CloudT::ConstPtr selected_target_cloud;
      double selected_radius = fine_gicp_target_radius_;

      if (adaptive_fine_gicp_enabled_) {
        bool selected = false;
        for (const double radius : adaptive_radii) {
          auto candidate_source_cloud = extract_cloud_neighborhood(
            source_cloud, Eigen::Vector3f::Zero(), radius, fine_gicp_z_radius_);
          auto candidate_target_cloud = extract_cloud_neighborhood(
            fine_gicp_map_cloud_, Eigen::Vector3f(origin.x(), origin.y(), origin.z()), radius,
            fine_gicp_z_radius_);
          auto feature_source_cloud = extract_cloud_neighborhood_with_z_band(
            source_cloud, Eigen::Vector3f::Zero(), radius, adaptive_fine_gicp_feature_z_min_,
            adaptive_fine_gicp_feature_z_max_);
          auto feature_target_cloud = extract_cloud_neighborhood_with_z_band(
            fine_gicp_map_cloud_, Eigen::Vector3f(origin.x(), origin.y(), origin.z()), radius,
            adaptive_fine_gicp_feature_z_min_, adaptive_fine_gicp_feature_z_max_);
          const auto source_metrics = evaluate_cloud_feature_metrics(
            feature_source_cloud, adaptive_fine_gicp_xy_cell_size_,
            adaptive_fine_gicp_min_source_points_, adaptive_fine_gicp_min_source_xy_cells_,
            adaptive_fine_gicp_min_scattering_, adaptive_fine_gicp_max_linearity_);
          const auto target_metrics = evaluate_cloud_feature_metrics(
            feature_target_cloud, adaptive_fine_gicp_xy_cell_size_,
            adaptive_fine_gicp_min_target_points_, adaptive_fine_gicp_min_target_xy_cells_,
            adaptive_fine_gicp_min_scattering_, adaptive_fine_gicp_max_linearity_);

          // 半径ごとにsource/target双方の局所形状を評価し、十分な最小範囲だけをGICPへ渡す。
          RCLCPP_INFO(
            get_logger(),
            "Fine GICP adaptive radius: attempt=%zu index=%zu/%zu radius=%.3f "
            "source(points=%zu cells=%zu lin=%.3f plan=%.3f scat=%.3f passed=%d) "
            "target(points=%zu cells=%zu lin=%.3f plan=%.3f scat=%.3f passed=%d)",
            initialization_attempt_count_, index + 1U, refine_count, radius,
            source_metrics.filtered_points, source_metrics.xy_cells, source_metrics.linearity,
            source_metrics.planarity, source_metrics.scattering, source_metrics.passed ? 1 : 0,
            target_metrics.filtered_points, target_metrics.xy_cells, target_metrics.linearity,
            target_metrics.planarity, target_metrics.scattering, target_metrics.passed ? 1 : 0);
          if (!source_metrics.passed || !target_metrics.passed) {
            continue;
          }

          selected_source_cloud = candidate_source_cloud;
          selected_target_cloud = candidate_target_cloud;
          selected_radius = radius;
          selected = true;
          RCLCPP_INFO(
            get_logger(),
            "Fine GICP adaptive radius selected: attempt=%zu index=%zu/%zu radius=%.3f",
            initialization_attempt_count_, index + 1U, refine_count, selected_radius);
          break;
        }

        if (!selected) {
          RCLCPP_WARN(
            get_logger(),
            "Fine GICP candidate skipped by feature gating: attempt=%zu index=%zu/%zu "
            "max_radius=%.3f coarse_fitness=%.6f",
            initialization_attempt_count_, index + 1U, refine_count, adaptive_radii.back(),
            coarse_result.registration.fitness);
          continue;
        }
      } else {
        selected_source_cloud = extract_cloud_neighborhood(
          source_cloud, Eigen::Vector3f::Zero(), fine_gicp_source_radius_, fine_gicp_z_radius_);
        selected_target_cloud = extract_cloud_neighborhood(
          fine_gicp_map_cloud_, Eigen::Vector3f(origin.x(), origin.y(), origin.z()),
          fine_gicp_target_radius_, fine_gicp_z_radius_);
      }

      if (selected_source_cloud->size() < static_cast<std::size_t>(min_registration_points_)) {
        RCLCPP_WARN(
          get_logger(),
          "Fine GICP candidate skipped: attempt=%zu index=%zu/%zu source_points=%zu "
          "min_registration_points=%d radius=%.3f coarse_fitness=%.6f",
          initialization_attempt_count_, index + 1U, refine_count, selected_source_cloud->size(),
          min_registration_points_, selected_radius, coarse_result.registration.fitness);
        continue;
      }
      if (selected_target_cloud->size() < static_cast<std::size_t>(min_registration_points_)) {
        RCLCPP_WARN(
          get_logger(),
          "Fine GICP candidate skipped: attempt=%zu index=%zu/%zu target_points=%zu "
          "min_registration_points=%d radius=%.3f coarse_fitness=%.6f",
          initialization_attempt_count_, index + 1U, refine_count, selected_target_cloud->size(),
          min_registration_points_, selected_radius, coarse_result.registration.fitness);
        continue;
      }

      // 粗いNDTのposeを初期値にして、局所範囲だけをGICPで精密化する。
      const auto gicp_start = std::chrono::steady_clock::now();
      auto registration = run_gicp(
        selected_source_cloud, selected_target_cloud,
        coarse_result.registration.target_from_source);
      const double gicp_ms = elapsed_milliseconds(
        gicp_start, std::chrono::steady_clock::now());
      const bool accepted = registration.converged &&
        registration.fitness <= fine_gicp_fitness_threshold_;
      const auto result_origin = registration.target_from_source.getOrigin();
      RCLCPP_INFO(
        get_logger(),
        "Fine GICP candidate: attempt=%zu index=%zu/%zu radius=%.3f source_points=%zu "
        "target_points=%zu result=(%.3f, %.3f, %.3f, %.6f) duration_ms=%.3f converged=%d "
        "fitness=%.6f threshold=%.6f accepted=%d coarse_fitness=%.6f",
        initialization_attempt_count_, index + 1U, refine_count, selected_radius,
        selected_source_cloud->size(), selected_target_cloud->size(), result_origin.x(),
        result_origin.y(), result_origin.z(), yaw_from_transform(registration.target_from_source),
        gicp_ms,
        registration.converged ? 1 : 0, registration.fitness, fine_gicp_fitness_threshold_,
        accepted ? 1 : 0, coarse_result.registration.fitness);
      if (accepted) {
        refined_results.push_back(
          InitialResult{coarse_result.candidate, registration, coarse_result.registration.fitness,
            true, coarse_result.distance_score, coarse_result.refined_with_distance});

        // coarse NDT後のGICPが十分良ければ、その時点で最終候補として扱い残り精密化を省略する。
        if (should_early_accept_gicp(registration)) {
          RCLCPP_INFO(
            get_logger(),
            "Fine GICP early accepted: attempt=%zu accepted_candidates=%zu "
            "fitness=%.6f threshold=%.6f",
            initialization_attempt_count_, refined_results.size(), registration.fitness,
            fine_gicp_early_accept_fitness_);
          break;
        }
      }
    }

    std::sort(
      refined_results.begin(), refined_results.end(), [this](const auto & lhs, const auto & rhs) {
        return is_initial_result_better(lhs, rhs);
      });
    return refined_results;
  }

  template<typename InitialResultT>
  std::string initialization_summary(const InitialResultT & result) const
  {
    std::ostringstream stream;
    stream << "initialized by Scan Context";
    if (result.refined_with_distance) {
      stream << " + distance transform";
    }
    stream << " + coarse NDT";
    if (result.refined_with_gicp) {
      stream << " + fine GICP";
    }
    if (result.refined_with_distance) {
      stream << " dt_score=" << result.distance_score.score
             << " dt_mean=" << result.distance_score.mean_distance
             << " dt_hit=" << result.distance_score.hit_ratio;
    }
    stream
      << " sc_score=" << result.candidate.score
      << " yaw=" << result.candidate.yaw
      << " coarse_fitness=" << result.coarse_fitness
      << " fitness=" << result.registration.fitness;
    return stream.str();
  }

  void compact_accumulated_scan()
  {
    accumulated_scan_lidar_init_ = voxel_downsample(
      accumulated_scan_lidar_init_, initial_ndt_scan_voxel_leaf_size_);
  }

  RegistrationResult run_ndt(
    const CloudT::ConstPtr & source_cloud, const CloudT::ConstPtr & target_cloud,
    const tf2::Transform & initial_guess)
  {
    RegistrationResult result;
    if (source_cloud->size() < static_cast<std::size_t>(min_registration_points_) ||
      target_cloud->size() < static_cast<std::size_t>(min_registration_points_))
    {
      return result;
    }

    try {
      // 粗いNDTはScan Context候補をGICPへ渡せる程度に整える役割に限定する。
      pcl::NormalDistributionsTransform<PointT, PointT> ndt;
      ndt.setInputSource(source_cloud);
      ndt.setInputTarget(target_cloud);
      ndt.setResolution(ndt_resolution_);
      ndt.setMaximumIterations(ndt_max_iterations_);
      ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
      ndt.setStepSize(ndt_step_size_);

      CloudT aligned_cloud;
      ndt.align(aligned_cloud, transform_to_eigen(initial_guess));
      result.converged = ndt.hasConverged();
      result.fitness = ndt.getFitnessScore();
      result.target_from_source = eigen_to_transform(ndt.getFinalTransformation());
      if (!std::isfinite(result.fitness) || !is_finite_transform(result.target_from_source)) {
        result.converged = false;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "NDT registration failed: %s", error.what());
      result.converged = false;
    }
    return result;
  }

  RegistrationResult run_gicp(
    const CloudT::ConstPtr & source_cloud, const CloudT::ConstPtr & target_cloud,
    const tf2::Transform & initial_guess)
  {
    RegistrationResult result;
    if (source_cloud->size() < static_cast<std::size_t>(min_registration_points_) ||
      target_cloud->size() < static_cast<std::size_t>(min_registration_points_))
    {
      return result;
    }

    try {
      // GICPは局所精密化だけに使い、対応点距離で外れ対応の影響を抑える。
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setInputSource(source_cloud);
      gicp.setInputTarget(target_cloud);
      gicp.setMaxCorrespondenceDistance(fine_gicp_max_correspondence_distance_);
      gicp.setMaximumIterations(fine_gicp_max_iterations_);
      gicp.setTransformationEpsilon(fine_gicp_transformation_epsilon_);

      CloudT aligned_cloud;
      gicp.align(aligned_cloud, transform_to_eigen(initial_guess));
      result.converged = gicp.hasConverged();
      result.fitness = gicp.getFitnessScore();
      result.target_from_source = eigen_to_transform(gicp.getFinalTransformation());
      if (!std::isfinite(result.fitness) || !is_finite_transform(result.target_from_source)) {
        result.converged = false;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "GICP registration failed: %s", error.what());
      result.converged = false;
    }
    return result;
  }

  void track_scan(
    const CloudT::ConstPtr & scan_cloud, const StampedOdometry & odometry,
    const rclcpp::Time & stamp)
  {
    const auto predicted_map_to_lidar_odom =
      last_map_to_lidar_init_ * odometry.lidar_init_from_lidar_odom;
    const auto origin = predicted_map_to_lidar_odom.getOrigin();
    auto local_map = extract_cloud_neighborhood(
      tracking_ndt_map_cloud_, Eigen::Vector3f(
        origin.x(), origin.y(),
        origin.z()), local_map_radius_,
      local_map_z_radius_);
    if (local_map->size() < static_cast<std::size_t>(min_registration_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Tracking skipped: local_map_points=%zu min_registration_points=%d "
        "predicted_origin=(%.3f, %.3f, %.3f)",
        local_map->size(), min_registration_points_, origin.x(), origin.y(), origin.z());
      handle_tracking_failure("local map has too few points for NDT");
      return;
    }

    const auto ndt_start = std::chrono::steady_clock::now();
    auto registration = run_ndt(scan_cloud, local_map, predicted_map_to_lidar_odom);
    const double ndt_ms = elapsed_milliseconds(ndt_start, std::chrono::steady_clock::now());
    if (!registration.converged || registration.fitness > ndt_fitness_threshold_) {
      std::ostringstream stream;
      stream << "tracking NDT rejected scan converged=" << registration.converged
             << " fitness=" << registration.fitness;
      RCLCPP_WARN(
        get_logger(),
        "Tracking NDT rejected: scan_points=%zu local_map_points=%zu duration_ms=%.3f "
        "converged=%d fitness=%.6f threshold=%.6f",
        scan_cloud->size(), local_map->size(), ndt_ms, registration.converged ? 1 : 0,
        registration.fitness, ndt_fitness_threshold_);
      handle_tracking_failure(stream.str());
      return;
    }

    // NDTはmap上の現在LiDAR poseを返すため、LIO-SAM odometryを戻してmap->lidar_initを更新する。
    last_map_to_lidar_init_ =
      registration.target_from_source * odometry.lidar_init_from_lidar_odom.inverse();
    tracking_failure_count_ = 0;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Tracking NDT accepted: scan_points=%zu local_map_points=%zu duration_ms=%.3f fitness=%.6f",
      scan_cloud->size(), local_map->size(), ndt_ms, registration.fitness);
    publish_pose_and_tf(stamp);
    publish_status(
      "tracking updated by local NDT fitness=" + std::to_string(registration.fitness));
  }

  void handle_tracking_failure(const std::string & reason)
  {
    ++tracking_failure_count_;
    std::ostringstream stream;
    stream << reason << " failure_count=" << tracking_failure_count_ << "/" <<
      tracking_failure_limit_;
    publish_status(stream.str());
    if (tracking_failure_count_ >= tracking_failure_limit_) {
      reset_accumulation();
      transition_to(LocalizationState::LOST, "tracking failure limit exceeded");
    }
  }

  void publish_pose_and_tf(const rclcpp::Time & stamp)
  {
    if (!is_finite_transform(last_map_to_lidar_init_)) {
      RCLCPP_WARN(get_logger(), "Skip localization publish because transform is invalid.");
      return;
    }

    pose_publisher_->publish(make_pose_message(last_map_to_lidar_init_, stamp, map_frame_));
    if (publish_tf_) {
      tf_broadcaster_.sendTransform(
        make_transform_message(last_map_to_lidar_init_, stamp, map_frame_, odom_frame_));
    }
    const auto origin = last_map_to_lidar_init_.getOrigin();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Published localization pose and TF: stamp=%.6f parent=%s child=%s publish_tf=%d "
      "translation=(%.3f, %.3f, %.3f) yaw=%.6f",
      stamp.seconds(), map_frame_.c_str(), odom_frame_.c_str(), publish_tf_ ? 1 : 0,
      origin.x(), origin.y(), origin.z(), yaw_from_transform(last_map_to_lidar_init_));
  }

  void transition_to(const LocalizationState next_state, const std::string & reason)
  {
    if (state_ != next_state) {
      RCLCPP_INFO(
        get_logger(), "Localization state transition: %s -> %s reason=%s",
        state_to_string(state_).c_str(), state_to_string(next_state).c_str(), reason.c_str());
      state_ = next_state;
    }
    publish_status(reason);
  }

  void publish_status(const std::string & reason)
  {
    std_msgs::msg::String message;
    message.data = state_to_string(state_) + ": " + reason;
    status_publisher_->publish(message);
    if (message.data != last_status_log_) {
      RCLCPP_INFO(get_logger(), "Localization status: %s", message.data.c_str());
      last_status_log_ = message.data;
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Localization status still: %s", message.data.c_str());
  }

  void log_diagnostics() const
  {
    const double accumulation_duration =
      (accumulation_start_stamp_.has_value() && latest_cloud_stamp_.has_value()) ?
      (*latest_cloud_stamp_ - *accumulation_start_stamp_).seconds() : 0.0;
    const double latest_odom_stamp = latest_odometry_.has_value() ?
      latest_odometry_->stamp.seconds() : 0.0;
    const double latest_cloud_stamp = latest_cloud_stamp_.has_value() ?
      latest_cloud_stamp_->seconds() : 0.0;

    RCLCPP_INFO(
      get_logger(),
      "Localization diagnostics: state=%s odom_count=%zu cloud_messages=%zu processed_clouds=%zu "
      "accumulated_points=%zu accumulation_duration=%.3f latest_odom_stamp=%.6f "
      "latest_cloud_stamp=%.6f attempts=%zu",
      state_to_string(
        state_).c_str(), odometry_count_, cloud_message_count_, processed_cloud_count_,
      accumulated_scan_lidar_init_->size(), accumulation_duration, latest_odom_stamp,
      latest_cloud_stamp, initialization_attempt_count_);
  }

  std::string pcd_map_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string map_preview_topic_;
  double scan_context_map_voxel_leaf_size_{};
  double scan_context_scan_voxel_leaf_size_{};
  double initial_ndt_map_voxel_leaf_size_{};
  double initial_ndt_scan_voxel_leaf_size_{};
  bool fine_gicp_enabled_{};
  int fine_gicp_candidate_count_{};
  double fine_gicp_map_voxel_leaf_size_{};
  double fine_gicp_scan_voxel_leaf_size_{};
  bool distance_match_enabled_{};
  double distance_match_grid_resolution_{};
  double distance_match_max_distance_{};
  double distance_match_map_z_min_{};
  double distance_match_map_z_max_{};
  double distance_match_scan_z_min_{};
  double distance_match_scan_z_max_{};
  double distance_match_search_xy_radius_{};
  double distance_match_search_xy_step_{};
  double distance_match_search_yaw_range_deg_{};
  double distance_match_search_yaw_step_deg_{};
  int distance_match_candidate_count_{};
  double distance_match_hit_distance_{};
  double distance_match_max_mean_distance_{};
  double distance_match_min_hit_ratio_{};
  double distance_match_max_far_ratio_{};
  double distance_match_max_out_of_bounds_ratio_{};
  double tracking_ndt_map_voxel_leaf_size_{};
  double tracking_ndt_scan_voxel_leaf_size_{};
  double initial_accumulation_sec_{};
  int initial_candidate_count_{};
  int scan_context_rings_{};
  int scan_context_sectors_{};
  double scan_context_max_radius_{};
  double scan_context_grid_step_{};
  double scan_context_score_threshold_{};
  double scan_context_score_gap_threshold_{};
  double ndt_resolution_{};
  int ndt_max_iterations_{};
  double ndt_fitness_threshold_{};
  double ndt_fitness_gap_threshold_{};
  bool initial_ndt_early_accept_enabled_{};
  double initial_ndt_early_accept_fitness_{};
  int initial_ndt_early_accept_min_candidates_{};
  double ndt_transformation_epsilon_{};
  double ndt_step_size_{};
  bool continuous_localization_enabled_{};
  int tracking_failure_limit_{};
  bool publish_tf_{};
  double initial_ndt_source_radius_{};
  double initial_ndt_target_radius_{};
  double initial_ndt_z_radius_{};
  double fine_gicp_source_radius_{};
  double fine_gicp_target_radius_{};
  double fine_gicp_z_radius_{};
  double fine_gicp_max_correspondence_distance_{};
  int fine_gicp_max_iterations_{};
  double fine_gicp_transformation_epsilon_{};
  double fine_gicp_fitness_threshold_{};
  double fine_gicp_fitness_gap_threshold_{};
  bool fine_gicp_early_accept_enabled_{};
  double fine_gicp_early_accept_fitness_{};
  bool adaptive_fine_gicp_enabled_{};
  double adaptive_fine_gicp_radius_1_{};
  double adaptive_fine_gicp_radius_2_{};
  double adaptive_fine_gicp_radius_3_{};
  double adaptive_fine_gicp_radius_4_{};
  double adaptive_fine_gicp_feature_z_min_{};
  double adaptive_fine_gicp_feature_z_max_{};
  std::size_t adaptive_fine_gicp_min_source_points_{};
  std::size_t adaptive_fine_gicp_min_target_points_{};
  std::size_t adaptive_fine_gicp_min_source_xy_cells_{};
  std::size_t adaptive_fine_gicp_min_target_xy_cells_{};
  double adaptive_fine_gicp_xy_cell_size_{};
  double adaptive_fine_gicp_min_scattering_{};
  double adaptive_fine_gicp_max_linearity_{};
  double local_map_radius_{};
  double local_map_z_radius_{};
  double initial_pose_z_{};
  int min_registration_points_{};
  int min_scan_context_points_{};
  int min_scan_context_bins_{};
  double initial_max_odometry_translation_{};

  CloudT::Ptr scan_context_map_cloud_{new CloudT};
  CloudT::Ptr initial_ndt_map_cloud_{new CloudT};
  CloudT::Ptr fine_gicp_map_cloud_{new CloudT};
  CloudT::Ptr tracking_ndt_map_cloud_{new CloudT};
  DistanceFieldMap distance_field_map_;
  std::vector<MapContext> map_contexts_;
  CloudT::Ptr accumulated_scan_lidar_init_{new CloudT};
  std::optional<rclcpp::Time> accumulation_start_stamp_;
  std::optional<tf2::Transform> accumulation_start_odometry_;
  std::optional<StampedOdometry> latest_odometry_;
  std::optional<rclcpp::Time> latest_cloud_stamp_;
  tf2::Transform last_map_to_lidar_init_{tf2::Transform::getIdentity()};
  LocalizationState state_{LocalizationState::UNINITIALIZED};
  int tracking_failure_count_{};
  std::size_t odometry_count_{};
  std::size_t cloud_message_count_{};
  std::size_t processed_cloud_count_{};
  std::size_t initialization_attempt_count_{};
  std::string last_status_log_;

  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<PointCloud2>::SharedPtr map_preview_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ai_ship_robot_slam::PcdLocalizationNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "pcd_localization_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
