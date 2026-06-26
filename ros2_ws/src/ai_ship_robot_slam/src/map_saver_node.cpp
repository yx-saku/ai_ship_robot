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

#include <unistd.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ai_ship_robot_slam/map_saver_utils.hpp"

namespace ai_ship_robot_slam
{
namespace
{
using Odometry = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Trigger = std_srvs::srv::Trigger;

struct PendingCloud
{
  rclcpp::Time stamp;
  std::vector<XyziPoint> local_points;
};

struct StampedPose
{
  rclcpp::Time stamp;
  tf2::Transform path_from_lidar;
};

struct ScanCloud
{
  rclcpp::Time stamp;
  tf2::Transform path_from_lidar;
  std::vector<XyziPoint> local_points;
};

struct MapSubmap
{
  std::size_t anchor_index{};
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose anchor_pose;
  double anchor_yaw{};
  std::size_t scan_count{};
  std::vector<XyziPoint> localization_points;
  std::vector<ElevationCell> elevation_cells;
};

struct VoxelKey
{
  std::int64_t x{};
  std::int64_t y{};
  std::int64_t z{};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<std::int64_t>{}(key.x);
    const auto hy = std::hash<std::int64_t>{}(key.y);
    const auto hz = std::hash<std::int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

template<typename T>
T read_unaligned(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

const PointField * find_field(const PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

std::size_t datatype_size(const std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1U;
    case PointField::INT16:
    case PointField::UINT16:
      return 2U;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4U;
    case PointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

bool field_fits_point_step(const PointField & field, const std::uint32_t point_step)
{
  const auto size = datatype_size(field.datatype);
  return size > 0U && field.offset + size <= point_step;
}

float read_numeric_field(
  const PointCloud2 & cloud, const PointField & field, const std::size_t point_offset)
{
  const auto offset = point_offset + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      return static_cast<float>(read_unaligned<std::int8_t>(cloud.data, offset));
    case PointField::UINT8:
      return static_cast<float>(read_unaligned<std::uint8_t>(cloud.data, offset));
    case PointField::INT16:
      return static_cast<float>(read_unaligned<std::int16_t>(cloud.data, offset));
    case PointField::UINT16:
      return static_cast<float>(read_unaligned<std::uint16_t>(cloud.data, offset));
    case PointField::INT32:
      return static_cast<float>(read_unaligned<std::int32_t>(cloud.data, offset));
    case PointField::UINT32:
      return static_cast<float>(read_unaligned<std::uint32_t>(cloud.data, offset));
    case PointField::FLOAT32:
      return read_unaligned<float>(cloud.data, offset);
    case PointField::FLOAT64:
      return static_cast<float>(read_unaligned<double>(cloud.data, offset));
    default:
      return 0.0F;
  }
}

std::filesystem::path default_output_directory()
{
  if (const auto * workspace_root = std::getenv("AI_SHIP_ROBOT_WORKSPACE_ROOT")) {
    return std::filesystem::path(workspace_root) / "outputs" / "cloud_map";
  }
  return std::filesystem::current_path() / "outputs" / "cloud_map";
}

std::string timestamp_string()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

double time_difference_seconds(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return std::abs((lhs - rhs).seconds());
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
  return tf2::Transform(rotation, translation);
}

tf2::Transform transform_to_tf2(const geometry_msgs::msg::Transform & transform)
{
  tf2::Quaternion rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  tf2::Vector3 translation(
    transform.translation.x, transform.translation.y, transform.translation.z);
  return tf2::Transform(rotation, translation);
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  return yaw;
}

std::size_t retained_point_count(const std::vector<ElevationCell> & cells)
{
  return std::accumulate(
    cells.begin(), cells.end(), std::size_t{0},
    [](const auto total, const auto & cell) {return total + cell.count;});
}

std::string yaml_quote(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (const auto character : value) {
    if (character == '"' || character == '\\') {
      stream << '\\';
    }
    stream << character;
  }
  stream << '"';
  return stream.str();
}

std::string relative_path_string(const std::filesystem::path & path)
{
  return path.generic_string();
}
}  // namespace

class MapSaverNode : public rclcpp::Node
{
public:
  MapSaverNode()
  : Node("map_saver_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    output_directory_ = this->declare_parameter<std::string>(
      "output_directory", default_output_directory().string());
    cloud_topic_ = this->declare_parameter<std::string>(
      "cloud_topic", "/lio_sam/mapping/cloud_registered_raw");
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/lio_sam/mapping/odometry");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/lio_sam/mapping/path");
    odometry_sync_tolerance_sec_ = this->declare_parameter<double>(
      "odometry_sync_tolerance_sec", 0.15);
    cloud_buffer_duration_sec_ = this->declare_parameter<double>("cloud_buffer_duration_sec", 5.0);
    localization_voxel_leaf_size_ = this->declare_parameter<double>(
      "localization_voxel_leaf_size", 0.10);
    global_voxel_leaf_size_ = this->declare_parameter<double>("global_voxel_leaf_size", 0.10);
    elevation_cell_size_ = this->declare_parameter<double>("elevation_cell_size", 0.01);
    cell_z_cluster_gap_ = this->declare_parameter<double>("cell_z_cluster_gap", 0.03);
    ground_cluster_height_gap_ = this->declare_parameter<double>("ground_cluster_height_gap", 0.05);
    ground_cluster_min_cells_ = this->declare_parameter<int>("ground_cluster_min_cells", 10);
    preview_enabled_ = this->declare_parameter<bool>("preview_enabled", true);
    preview_topic_ = this->declare_parameter<std::string>(
      "preview_topic", "/map_saver/localization_map_preview");
    preview_publish_period_sec_ = this->declare_parameter<double>(
      "preview_publish_period_sec", 2.0);
    preview_voxel_leaf_size_ = this->declare_parameter<double>("preview_voxel_leaf_size", 0.10);

    // raw登録点群は未確定submap分だけ保持し、odometry同期後にsubmap確定処理へ流す。
    cloud_subscription_ = this->create_subscription<PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const PointCloud2::SharedPtr message) {this->handle_cloud(message);});
    odometry_subscription_ = this->create_subscription<Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const Odometry::SharedPtr message) {this->handle_odometry(message);});
    path_subscription_ = this->create_subscription<Path>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const Path::SharedPtr message) {this->handle_path(message);});

    // 保存APIは新仕様の/save_mapsだけを提供し、旧サービスとの互換口は持たない。
    save_service_ = this->create_service<Trigger>(
      "save_maps",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        this->save_maps(response);
      });

    if (preview_enabled_) {
      // previewは詳細rawではなくlocalization用粗点群から作り、RViz表示時の負荷を抑える。
      preview_publisher_ = this->create_publisher<PointCloud2>(
        preview_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      preview_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(preview_publish_period_sec_, 0.1))),
        [this]() {this->publish_preview();});
    }
  }

  ~MapSaverNode() override
  {
    if (submaps_.empty() && scan_clouds_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No accumulated map data to save on shutdown.");
      return;
    }
    if (map_saved_since_last_update_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Map outputs were already saved after the last update; skip shutdown save.");
      return;
    }

    // service呼び忘れ時のデータ消失を避けるため、終了時にも同じ保存処理を一度だけ試す。
    std::string message;
    if (save_maps_file(message)) {
      RCLCPP_INFO(this->get_logger(), "Saved map outputs on shutdown: %s", message.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to save map outputs on shutdown: %s",
        message.c_str());
    }
  }

private:
  void handle_cloud(const PointCloud2::SharedPtr message)
  {
    PendingCloud pending_cloud{rclcpp::Time(message->header.stamp), {}};
    if (!extract_cloud_points(*message, pending_cloud.local_points)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Drop map saver cloud because PointCloud2 fields are invalid: stamp=%.6f",
        pending_cloud.stamp.seconds());
      return;
    }

    // cloudはodometry同期までの短期間だけpendingに積み、同期後はscan bufferへ移動する。
    pending_clouds_.push_back(std::move(pending_cloud));
    map_saved_since_last_update_ = false;
    match_pending_clouds();
  }

  void handle_odometry(const Odometry::SharedPtr message)
  {
    path_frame_ = message->header.frame_id;
    odometry_buffer_.push_back(
      StampedPose{
        rclcpp::Time(message->header.stamp), pose_to_transform(message->pose.pose)});
    prune_odometry_buffer(odometry_buffer_.back().stamp);
    match_pending_clouds();
  }

  void handle_path(const Path::SharedPtr message)
  {
    if (message->poses.empty()) {
      return;
    }

    // pathはloop closure後に更新されるため、localization PCDのglobal再配置用に最新版を保持する。
    map_saved_since_last_update_ = false;
    path_frame_ = message->header.frame_id;
    latest_path_stamp_ = rclcpp::Time(message->header.stamp);
    latest_path_ = message->poses;
    if (latest_path_.size() < initial_keyframe_poses_.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received a shorter path; reset accumulated map saver state.");
      reset_accumulation();
    }
    for (std::size_t index = initial_keyframe_poses_.size(); index < latest_path_.size(); ++index) {
      initial_keyframe_poses_.push_back(latest_path_[index]);
    }
    finalize_ready_submaps();
  }

  void reset_accumulation()
  {
    initial_keyframe_poses_.clear();
    submaps_.clear();
    scan_clouds_.clear();
    pending_clouds_.clear();
    next_submap_anchor_index_ = 0;
  }

  bool extract_cloud_points(const PointCloud2 & cloud, std::vector<XyziPoint> & points)
  {
    const auto * x_field = find_field(cloud, "x");
    const auto * y_field = find_field(cloud, "y");
    const auto * z_field = find_field(cloud, "z");
    const auto * intensity_field = find_field(cloud, "intensity");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Cloud does not contain x/y/z fields.");
      return false;
    }
    if (!field_fits_point_step(*x_field, cloud.point_step) ||
      !field_fits_point_step(*y_field, cloud.point_step) ||
      !field_fits_point_step(*z_field, cloud.point_step) ||
      (intensity_field != nullptr && !field_fits_point_step(*intensity_field, cloud.point_step)))
    {
      return false;
    }

    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (cloud.point_step == 0U ||
      point_count * static_cast<std::size_t>(cloud.point_step) > cloud.data.size())
    {
      return false;
    }

    // organized/unorganizedのどちらでもPointCloud2のpoint_step単位でXYZIだけを抽出する。
    points.clear();
    points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      const auto point_offset = index * cloud.point_step;
      XyziPoint point;
      point.x = read_numeric_field(cloud, *x_field, point_offset);
      point.y = read_numeric_field(cloud, *y_field, point_offset);
      point.z = read_numeric_field(cloud, *z_field, point_offset);
      point.intensity = intensity_field == nullptr ? 0.0F : read_numeric_field(
        cloud, *intensity_field, point_offset);
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        points.push_back(point);
      }
    }
    return true;
  }

  void prune_odometry_buffer(const rclcpp::Time & newest_stamp)
  {
    // 未同期cloudを救える範囲だけodometryを残し、古いpose bufferの増加を抑える。
    while (!odometry_buffer_.empty() &&
      (newest_stamp - odometry_buffer_.front().stamp).seconds() > cloud_buffer_duration_sec_)
    {
      odometry_buffer_.erase(odometry_buffer_.begin());
    }
  }

  std::optional<std::size_t> find_nearest_odometry_index(const rclcpp::Time & stamp) const
  {
    if (odometry_buffer_.empty()) {
      return std::nullopt;
    }

    double best_difference = std::numeric_limits<double>::max();
    std::optional<std::size_t> best_index;
    for (std::size_t index = 0; index < odometry_buffer_.size(); ++index) {
      const auto difference = time_difference_seconds(odometry_buffer_[index].stamp, stamp);
      if (difference < best_difference) {
        best_difference = difference;
        best_index = index;
      }
    }

    if (best_index.has_value() && best_difference <= odometry_sync_tolerance_sec_) {
      return best_index;
    }
    return std::nullopt;
  }

  void match_pending_clouds()
  {
    bool matched_any = false;
    auto cloud_iter = pending_clouds_.begin();
    while (cloud_iter != pending_clouds_.end()) {
      const auto odometry_index = find_nearest_odometry_index(cloud_iter->stamp);
      if (odometry_index.has_value()) {
        scan_clouds_.push_back(
          ScanCloud{
            cloud_iter->stamp,
            odometry_buffer_[*odometry_index].path_from_lidar,
            std::move(cloud_iter->local_points)});
        cloud_iter = pending_clouds_.erase(cloud_iter);
        matched_any = true;
        continue;
      }
      if (!odometry_buffer_.empty() &&
        (odometry_buffer_.back().stamp - cloud_iter->stamp).seconds() >
        odometry_sync_tolerance_sec_)
      {
        double best_difference = std::numeric_limits<double>::max();
        rclcpp::Time best_stamp = odometry_buffer_.front().stamp;
        for (const auto & odometry : odometry_buffer_) {
          const auto difference = time_difference_seconds(odometry.stamp, cloud_iter->stamp);
          if (difference < best_difference) {
            best_difference = difference;
            best_stamp = odometry.stamp;
          }
        }
        RCLCPP_WARN(
          this->get_logger(),
          "Drop map saver cloud because synchronized odometry was not found: "
          "cloud_stamp=%.6f best_odom_stamp=%.6f best_diff=%.6f tolerance=%.6f "
          "odom_oldest=%.6f odom_latest=%.6f pending_clouds=%zu points=%zu",
          cloud_iter->stamp.seconds(), best_stamp.seconds(), best_difference,
          odometry_sync_tolerance_sec_, odometry_buffer_.front().stamp.seconds(),
          odometry_buffer_.back().stamp.seconds(), pending_clouds_.size(),
          cloud_iter->local_points.size());
        cloud_iter = pending_clouds_.erase(cloud_iter);
        continue;
      }
      ++cloud_iter;
    }

    if (matched_any) {
      std::sort(
        scan_clouds_.begin(), scan_clouds_.end(), [](const auto & lhs, const auto & rhs) {
          return (lhs.stamp - rhs.stamp).seconds() < 0.0;
        });
      finalize_ready_submaps();
    }
  }

  std::vector<XyziPoint> apply_voxel_filter(
    const std::vector<XyziPoint> & points, const double leaf_size) const
  {
    if (leaf_size <= 0.0) {
      return points;
    }

    // localization向けの代表点だけをvoxel単位で残し、保存PCDとpreviewの点数を抑える。
    std::unordered_map<VoxelKey, XyziPoint, VoxelKeyHash> voxel_points;
    voxel_points.reserve(points.size());
    for (const auto & point : points) {
      if (!rclcpp::ok()) {
        return {};
      }
      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(point.x / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.y / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.z / leaf_size))};
      if (voxel_points.find(key) == voxel_points.end()) {
        voxel_points.emplace(key, point);
      }
    }

    std::vector<XyziPoint> filtered_points;
    filtered_points.reserve(voxel_points.size());
    for (const auto & [key, point] : voxel_points) {
      (void)key;
      filtered_points.push_back(point);
    }
    return filtered_points;
  }

  ElevationExtractionParams elevation_params() const
  {
    ElevationExtractionParams params;
    params.cell_size = elevation_cell_size_;
    params.cell_z_cluster_gap = cell_z_cluster_gap_;
    params.ground_cluster_height_gap = ground_cluster_height_gap_;
    params.ground_cluster_min_cells = static_cast<std::size_t>(
      std::max(1, ground_cluster_min_cells_));
    return params;
  }

  MapSubmap create_submap(
    const std::size_t anchor_index, const std::optional<rclcpp::Time> & end_stamp) const
  {
    MapSubmap submap;
    submap.anchor_index = anchor_index;
    submap.stamp = rclcpp::Time(initial_keyframe_poses_[anchor_index].header.stamp);
    submap.anchor_pose = initial_keyframe_poses_[anchor_index].pose;
    submap.anchor_yaw = yaw_from_pose(submap.anchor_pose);
    const auto path_from_anchor_initial = pose_to_transform(submap.anchor_pose);
    std::vector<XyziPoint> merged_localization_points;
    std::vector<ElevationPoint> elevation_points;

    // 同一keyframe区間のraw scanを、用途別のsubmap点群へ同時変換する。
    for (const auto & scan : scan_clouds_) {
      if ((scan.stamp - submap.stamp).seconds() < 0.0) {
        continue;
      }
      if (end_stamp.has_value() && (scan.stamp - *end_stamp).seconds() >= 0.0) {
        continue;
      }

      const auto anchor_from_scan = path_from_anchor_initial.inverse() * scan.path_from_lidar;
      ++submap.scan_count;
      merged_localization_points.reserve(
        merged_localization_points.size() + scan.local_points.size());
      elevation_points.reserve(elevation_points.size() + scan.local_points.size());
      for (const auto & local_point : scan.local_points) {
        const tf2::Vector3 source(local_point.x, local_point.y, local_point.z);
        const tf2::Vector3 anchor_local_point = anchor_from_scan * source;
        merged_localization_points.push_back(
          XyziPoint{
            static_cast<float>(anchor_local_point.x()),
            static_cast<float>(anchor_local_point.y()),
            static_cast<float>(anchor_local_point.z()),
            local_point.intensity});

        const tf2::Vector3 path_point = scan.path_from_lidar * source;
        elevation_points.push_back(
          to_anchor_yaw_local_map_z(
            path_point.x(), path_point.y(), path_point.z(), submap.anchor_pose.position.x,
            submap.anchor_pose.position.y, submap.anchor_yaw));
      }
    }

    submap.localization_points = apply_voxel_filter(
      merged_localization_points, localization_voxel_leaf_size_);
    submap.elevation_cells = extract_elevation_cells(elevation_points, elevation_params());
    return submap;
  }

  void erase_scans_before(const rclcpp::Time & stamp)
  {
    // 確定済みsubmapに変換したraw scanだけを破棄し、過去全期間のraw点群保持を避ける。
    scan_clouds_.erase(
      std::remove_if(
        scan_clouds_.begin(), scan_clouds_.end(),
        [&stamp](const ScanCloud & scan) {return (scan.stamp - stamp).seconds() < 0.0;}),
      scan_clouds_.end());
  }

  void finalize_ready_submaps()
  {
    while (next_submap_anchor_index_ + 1 < initial_keyframe_poses_.size()) {
      const auto end_stamp = rclcpp::Time(
        initial_keyframe_poses_[next_submap_anchor_index_ + 1].header.stamp);
      if (scan_clouds_.empty() ||
        (scan_clouds_.back().stamp - end_stamp).seconds() < -odometry_sync_tolerance_sec_)
      {
        break;
      }

      auto submap = create_submap(next_submap_anchor_index_, end_stamp);
      RCLCPP_INFO(
        this->get_logger(),
        "Finalized map submap: anchor=%zu scans=%zu localization_points=%zu "
        "elevation_cells=%zu elevation_points=%zu",
        submap.anchor_index, submap.scan_count, submap.localization_points.size(),
        submap.elevation_cells.size(), retained_point_count(submap.elevation_cells));
      if (!submap.localization_points.empty() || !submap.elevation_cells.empty()) {
        submaps_.push_back(std::move(submap));
      }
      erase_scans_before(end_stamp);
      ++next_submap_anchor_index_;
    }
  }

  void save_maps(const Trigger::Response::SharedPtr response)
  {
    response->success = save_maps_file(response->message);
  }

  bool resolve_target_transform(
    tf2::Transform & transform, std::string & frame_id,
    std::string & message)
  {
    transform.setIdentity();
    frame_id = path_frame_;
    if (path_frame_.empty()) {
      message = "path frame is empty";
      return false;
    }
    if (target_frame_.empty() || target_frame_ == path_frame_) {
      return true;
    }

    try {
      const auto stamped_transform = tf_buffer_.lookupTransform(
        target_frame_, path_frame_, latest_path_stamp_, rclcpp::Duration::from_seconds(0.2));
      transform = transform_to_tf2(stamped_transform.transform);
      frame_id = target_frame_;
      return true;
    } catch (const tf2::TransformException & error) {
      message = "failed to transform path frame to target frame: " + std::string(error.what());
      return false;
    }
  }

  void append_submap_global_points(
    const MapSubmap & submap,
    const tf2::Transform & target_from_path,
    std::vector<XyziPoint> & global_points) const
  {
    if (submap.anchor_index >= latest_path_.size()) {
      return;
    }

    // localization submapはanchor local保持なので、保存時だけtarget frameへ再配置する。
    const auto path_from_anchor_latest = pose_to_transform(latest_path_[submap.anchor_index].pose);
    const auto target_from_anchor = target_from_path * path_from_anchor_latest;
    global_points.reserve(global_points.size() + submap.localization_points.size());
    for (const auto & local_point : submap.localization_points) {
      if (!rclcpp::ok()) {
        return;
      }
      const tf2::Vector3 transformed = target_from_anchor *
        tf2::Vector3(local_point.x, local_point.y, local_point.z);
      global_points.push_back(
        XyziPoint{
          static_cast<float>(transformed.x()),
          static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z()),
          local_point.intensity});
    }
  }

  std::vector<XyziPoint> build_global_points(
    const std::vector<const MapSubmap *> & output_submaps,
    const tf2::Transform & target_from_path) const
  {
    std::vector<XyziPoint> global_points;
    for (const auto * submap : output_submaps) {
      if (!rclcpp::ok()) {
        return {};
      }
      append_submap_global_points(*submap, target_from_path, global_points);
    }
    return global_points;
  }

  std::optional<MapSubmap> create_provisional_submap() const
  {
    if (next_submap_anchor_index_ >= initial_keyframe_poses_.size()) {
      return std::nullopt;
    }
    auto submap = create_submap(next_submap_anchor_index_, std::nullopt);
    if (submap.scan_count == 0U ||
      (submap.localization_points.empty() && submap.elevation_cells.empty()))
    {
      return std::nullopt;
    }
    return submap;
  }

  std::filesystem::path next_output_directory_path() const
  {
    const auto base_directory = std::filesystem::path(output_directory_);
    const auto base_name = "map_" + timestamp_string();
    for (int suffix = 0; suffix < 100; ++suffix) {
      const auto name = suffix == 0 ? base_name : base_name + "_" + std::to_string(suffix);
      const auto candidate = base_directory / name;
      if (!std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
    return base_directory / (base_name + "_" + std::to_string(::getpid()));
  }

  bool write_elevation_csv(
    const std::filesystem::path & path,
    const std::vector<ElevationCell> & cells,
    std::string & message) const
  {
    std::ofstream file(path);
    if (!file) {
      message = "failed to open elevation CSV: " + path.string();
      return false;
    }

    // 後段処理が列順に依存できるよう、CSVヘッダは計画で定義した固定形式にする。
    file << "ix,iy,x,y,count,z_min,z_max,z_mean,z_m2\n";
    file << std::fixed << std::setprecision(9);
    for (const auto & cell : cells) {
      file << cell.ix << ',' << cell.iy << ',' << cell.x << ',' << cell.y << ',' << cell.count <<
        ','
           << cell.z_min << ',' << cell.z_max << ',' << cell.z_mean << ',' << cell.z_m2 << '\n';
    }
    if (!file) {
      message = "failed to write elevation CSV: " + path.string();
      return false;
    }
    return true;
  }

  bool write_manifest(
    const std::filesystem::path & path,
    const std::string & created_at,
    const std::string & output_frame,
    const std::string & localization_pcd_relative_path,
    const std::vector<const MapSubmap *> & output_submaps,
    const std::vector<std::string> & csv_relative_paths,
    std::string & message) const
  {
    std::ofstream file(path);
    if (!file) {
      message = "failed to open elevation manifest: " + path.string();
      return false;
    }

    // manifestにはsubmap座標系と抽出パラメータを明示し、後段のglobal配置で解釈がぶれないようにする。
    file << "format_version: 1\n";
    file << "created_at: " << yaml_quote(created_at) << "\n";
    file << "input_topic: " << yaml_quote(cloud_topic_) << "\n";
    file << "target_frame: " << yaml_quote(output_frame) << "\n";
    file << "submap_coordinate: \"anchor_yaw_local_xy_map_z\"\n";
    file << "localization_voxel_leaf_size: " << localization_voxel_leaf_size_ << "\n";
    file << "global_voxel_leaf_size: " << global_voxel_leaf_size_ << "\n";
    file << "elevation_cell_size: " << elevation_cell_size_ << "\n";
    file << "cell_z_cluster_gap: " << cell_z_cluster_gap_ << "\n";
    file << "ground_cluster_height_gap: " << ground_cluster_height_gap_ << "\n";
    file << "ground_cluster_min_cells: " << std::max(1, ground_cluster_min_cells_) << "\n";
    file << "localization_pcd: " << yaml_quote(localization_pcd_relative_path) << "\n";
    file << "submaps:\n";
    for (std::size_t index = 0; index < output_submaps.size(); ++index) {
      const auto & submap = *output_submaps[index];
      const auto stamp_nanoseconds = submap.stamp.nanoseconds();
      const auto stamp_sec = stamp_nanoseconds / 1000000000LL;
      const auto stamp_nsec = stamp_nanoseconds % 1000000000LL;
      const auto & pose = submap.anchor_pose;
      file << "  - index: " << index << "\n";
      file << "    stamp:\n";
      file << "      sec: " << stamp_sec << "\n";
      file << "      nanosec: " << stamp_nsec << "\n";
      file << "    stamp_seconds: " << std::fixed << std::setprecision(9) <<
        submap.stamp.seconds() << "\n";
      file << "    anchor_index: " << submap.anchor_index << "\n";
      file << "    scan_count: " << submap.scan_count << "\n";
      file << "    retained_cell_count: " << submap.elevation_cells.size() << "\n";
      file << "    retained_point_count: " << retained_point_count(submap.elevation_cells) << "\n";
      file << "    csv: " << yaml_quote(csv_relative_paths[index]) << "\n";
      file << "    anchor_pose:\n";
      file << "      position:\n";
      file << "        x: " << pose.position.x << "\n";
      file << "        y: " << pose.position.y << "\n";
      file << "        z: " << pose.position.z << "\n";
      file << "      orientation:\n";
      file << "        x: " << pose.orientation.x << "\n";
      file << "        y: " << pose.orientation.y << "\n";
      file << "        z: " << pose.orientation.z << "\n";
      file << "        w: " << pose.orientation.w << "\n";
      file << "      yaw: " << submap.anchor_yaw << "\n";
    }
    if (!file) {
      message = "failed to write elevation manifest: " + path.string();
      return false;
    }
    return true;
  }

  bool write_outputs(
    const std::filesystem::path & temporary_directory,
    const std::string & created_at,
    const std::string & output_frame,
    const std::vector<const MapSubmap *> & output_submaps,
    const std::vector<XyziPoint> & localization_points,
    std::string & message) const
  {
    const auto localization_pcd_relative_path = std::filesystem::path("localization_map.pcd");
    const auto manifest_relative_path = std::filesystem::path("elevation_manifest.yaml");
    const auto submap_directory_relative_path = std::filesystem::path("elevation_submaps");
    const auto submap_directory = temporary_directory / submap_directory_relative_path;
    std::error_code error;
    std::filesystem::create_directories(submap_directory, error);
    if (error) {
      message = "failed to create elevation submap directory: " + error.message();
      return false;
    }

    const auto localization_pcd_path = temporary_directory / localization_pcd_relative_path;
    if (!write_binary_xyzi_pcd(localization_pcd_path, localization_points, message)) {
      return false;
    }

    std::vector<std::string> csv_relative_paths;
    csv_relative_paths.reserve(output_submaps.size());
    for (std::size_t index = 0; index < output_submaps.size(); ++index) {
      std::ostringstream file_name;
      file_name << "submap_" << std::setw(6) << std::setfill('0') << index << ".csv";
      const auto csv_relative_path = submap_directory_relative_path / file_name.str();
      if (!write_elevation_csv(
          temporary_directory / csv_relative_path, output_submaps[index]->elevation_cells, message))
      {
        return false;
      }
      csv_relative_paths.push_back(relative_path_string(csv_relative_path));
    }

    return write_manifest(
      temporary_directory / manifest_relative_path, created_at, output_frame,
      relative_path_string(
        localization_pcd_relative_path), output_submaps, csv_relative_paths, message);
  }

  bool save_maps_file(std::string & message)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Map outputs save started: finalized_submaps=%zu scans=%zu path_poses=%zu",
      submaps_.size(), scan_clouds_.size(), latest_path_.size());
    finalize_ready_submaps();
    if (latest_path_.empty()) {
      message = "latest path is empty";
      return false;
    }

    tf2::Transform target_from_path;
    std::string output_frame;
    if (!resolve_target_transform(target_from_path, output_frame, message)) {
      return false;
    }

    std::optional<MapSubmap> provisional_submap = create_provisional_submap();
    std::vector<const MapSubmap *> output_submaps;
    output_submaps.reserve(submaps_.size() + (provisional_submap.has_value() ? 1U : 0U));
    for (const auto & submap : submaps_) {
      output_submaps.push_back(&submap);
    }
    if (provisional_submap.has_value()) {
      output_submaps.push_back(&*provisional_submap);
    }
    if (output_submaps.empty()) {
      message = "no accumulated submaps to save";
      return false;
    }

    auto localization_points = build_global_points(output_submaps, target_from_path);
    if (!rclcpp::ok()) {
      message = "map outputs save cancelled while building localization points";
      return false;
    }
    localization_points = apply_voxel_filter(localization_points, global_voxel_leaf_size_);
    if (!rclcpp::ok()) {
      message = "map outputs save cancelled while voxel filtering localization points";
      return false;
    }
    if (localization_points.empty()) {
      message = "no localization points to save";
      return false;
    }

    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {
      message = "failed to create output directory: " + error.message();
      return false;
    }

    const auto created_at = timestamp_string();
    const auto output_directory = next_output_directory_path();
    const auto temporary_directory = output_directory.parent_path() /
      ("." + output_directory.filename().string() + ".tmp." + std::to_string(::getpid()));
    std::filesystem::remove_all(temporary_directory, error);
    error.clear();
    std::filesystem::create_directories(temporary_directory, error);
    if (error) {
      message = "failed to create temporary output directory: " + error.message();
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Writing map outputs: tmp_dir=%s submaps=%zu localization_points=%zu",
      temporary_directory.string().c_str(), output_submaps.size(), localization_points.size());
    if (!write_outputs(
        temporary_directory, created_at, output_frame, output_submaps, localization_points,
        message))
    {
      std::filesystem::remove_all(temporary_directory, error);
      return false;
    }

    error.clear();
    std::filesystem::rename(temporary_directory, output_directory, error);
    if (error) {
      std::filesystem::remove_all(temporary_directory, error);
      message = "failed to rename temporary map output directory: " + error.message();
      return false;
    }

    const auto localization_pcd_path = output_directory / "localization_map.pcd";
    const auto manifest_path = output_directory / "elevation_manifest.yaml";
    message = "output_dir=" + output_directory.string() +
      " localization_pcd=" + localization_pcd_path.string() +
      " manifest=" + manifest_path.string() +
      " submaps=" + std::to_string(output_submaps.size()) +
      " localization_points=" + std::to_string(localization_points.size()) +
      " frame=" + output_frame;
    map_saved_since_last_update_ = true;
    RCLCPP_INFO(this->get_logger(), "Map outputs save finished: %s", message.c_str());
    return true;
  }

  PointCloud2 build_point_cloud2_message(
    const std::vector<XyziPoint> & points, const std::string & frame_id) const
  {
    PointCloud2 message;
    message.header.stamp = this->now();
    message.header.frame_id = frame_id;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = sizeof(XyziPoint);
    message.row_step = message.point_step * message.width;
    message.fields.resize(4);
    message.fields[0].name = "x";
    message.fields[0].offset = 0U;
    message.fields[0].datatype = PointField::FLOAT32;
    message.fields[0].count = 1U;
    message.fields[1].name = "y";
    message.fields[1].offset = 4U;
    message.fields[1].datatype = PointField::FLOAT32;
    message.fields[1].count = 1U;
    message.fields[2].name = "z";
    message.fields[2].offset = 8U;
    message.fields[2].datatype = PointField::FLOAT32;
    message.fields[2].count = 1U;
    message.fields[3].name = "intensity";
    message.fields[3].offset = 12U;
    message.fields[3].datatype = PointField::FLOAT32;
    message.fields[3].count = 1U;

    // XYZIだけの連続配置でPointCloud2を組み立て、preview publishをPCL非依存に保つ。
    message.data.resize(points.size() * sizeof(XyziPoint));
    if (!points.empty()) {
      std::memcpy(message.data.data(), points.data(), message.data.size());
    }
    return message;
  }

  bool build_preview_message(PointCloud2 & message, std::string & error_message)
  {
    finalize_ready_submaps();
    if (latest_path_.empty()) {
      error_message = "latest path is empty";
      return false;
    }

    tf2::Transform target_from_path;
    std::string output_frame;
    if (!resolve_target_transform(target_from_path, output_frame, error_message)) {
      return false;
    }

    std::optional<MapSubmap> provisional_submap = create_provisional_submap();
    std::vector<const MapSubmap *> preview_submaps;
    preview_submaps.reserve(submaps_.size() + (provisional_submap.has_value() ? 1U : 0U));
    for (const auto & submap : submaps_) {
      preview_submaps.push_back(&submap);
    }
    if (provisional_submap.has_value()) {
      preview_submaps.push_back(&*provisional_submap);
    }

    auto preview_points = apply_voxel_filter(
      build_global_points(preview_submaps, target_from_path), preview_voxel_leaf_size_);
    if (preview_points.empty()) {
      error_message = "no accumulated localization points for preview";
      return false;
    }

    message = build_point_cloud2_message(preview_points, output_frame);
    return true;
  }

  void publish_preview()
  {
    if (preview_publisher_ == nullptr) {
      return;
    }
    if (preview_publisher_->get_subscription_count() == 0U) {
      return;
    }

    PointCloud2 message;
    std::string error_message;
    if (!build_preview_message(message, error_message)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Skip map preview publish: %s", error_message.c_str());
      return;
    }

    preview_publisher_->publish(message);
  }

  std::string target_frame_;
  std::string output_directory_;
  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string preview_topic_;
  std::string path_frame_;
  double odometry_sync_tolerance_sec_{};
  double cloud_buffer_duration_sec_{};
  double localization_voxel_leaf_size_{};
  double global_voxel_leaf_size_{};
  double elevation_cell_size_{};
  double cell_z_cluster_gap_{};
  double ground_cluster_height_gap_{};
  double preview_publish_period_sec_{};
  double preview_voxel_leaf_size_{};
  int ground_cluster_min_cells_{};
  bool preview_enabled_{true};
  bool map_saved_since_last_update_{false};
  std::size_t next_submap_anchor_index_{};
  rclcpp::Time latest_path_stamp_{};
  std::vector<geometry_msgs::msg::PoseStamped> latest_path_;
  std::vector<geometry_msgs::msg::PoseStamped> initial_keyframe_poses_;
  std::vector<PendingCloud> pending_clouds_;
  std::vector<StampedPose> odometry_buffer_;
  std::vector<ScanCloud> scan_clouds_;
  std::vector<MapSubmap> submaps_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<Path>::SharedPtr path_subscription_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Publisher<PointCloud2>::SharedPtr preview_publisher_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::MapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
