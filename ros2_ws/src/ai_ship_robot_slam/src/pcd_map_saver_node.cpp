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
#include <optional>
#include <sstream>
#include <string>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ai_ship_robot_slam
{
namespace
{
using Odometry = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Trigger = std_srvs::srv::Trigger;

struct CloudPoint
{
  float x{};
  float y{};
  float z{};
  float intensity{};
};

struct PendingCloud
{
  rclcpp::Time stamp;
  std::vector<CloudPoint> local_points;
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
  std::vector<CloudPoint> local_points;
};

struct SubmapCloud
{
  std::size_t anchor_index{};
  rclcpp::Time stamp;
  std::size_t scan_count{};
  std::vector<CloudPoint> local_points;
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
}  // namespace

class PcdMapSaverNode : public rclcpp::Node
{
public:
  PcdMapSaverNode()
  : Node("pcd_map_saver_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    output_directory_ = this->declare_parameter<std::string>(
      "output_directory", default_output_directory().string());
    cloud_topic_ = this->declare_parameter<std::string>(
      "cloud_topic", "/lio_sam/mapping/cloud_registered_hybrid");
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/lio_sam/mapping/odometry");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/lio_sam/mapping/path");
    odometry_sync_tolerance_sec_ = this->declare_parameter<double>(
      "odometry_sync_tolerance_sec", 0.15);
    cloud_buffer_duration_sec_ = this->declare_parameter<double>("cloud_buffer_duration_sec", 5.0);
    submap_voxel_leaf_size_ = this->declare_parameter<double>("submap_voxel_leaf_size", 0.01);
    global_voxel_leaf_size_ = this->declare_parameter<double>("global_voxel_leaf_size", 0.0);

    // 全scan点群を一時保持し、odometry pose同期後にkeyframe区間submapへ圧縮する。
    cloud_subscription_ = this->create_subscription<PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const PointCloud2::SharedPtr message) { this->handle_cloud(message); });
    odometry_subscription_ = this->create_subscription<Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const Odometry::SharedPtr message) { this->handle_odometry(message); });
    path_subscription_ = this->create_subscription<Path>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const Path::SharedPtr message) { this->handle_path(message); });

    // 保存操作は固定サービス名に限定し、外部から必要なタイミングでPCDを書き出す。
    save_service_ = this->create_service<Trigger>(
      "save_pcd_map",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        this->save_map(response);
      });
  }

  ~PcdMapSaverNode() override
  {
    if (submaps_.empty() && scan_clouds_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No accumulated scan cloud points to save on shutdown.");
      return;
    }
    // 明示保存後の終了では、巨大PCDの再書き出しでshutdownが詰まることを避ける。
    if (map_saved_since_last_update_) {
      RCLCPP_INFO(this->get_logger(), "PCD map was already saved after the last update; skip shutdown save.");
      return;
    }

    // 終了時にも最新pathで再配置したmapを保存し、サービス呼び忘れによる地図消失を防ぐ。
    std::string message;
    if (save_map_file(message)) {
      RCLCPP_INFO(this->get_logger(), "Saved PCD map on shutdown: %s", message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD map on shutdown: %s", message.c_str());
    }
  }

private:
  void handle_cloud(const PointCloud2::SharedPtr message)
  {
    PendingCloud pending_cloud{rclcpp::Time(message->header.stamp), {}};
    if (!extract_cloud_points(*message, pending_cloud.local_points)) {
      RCLCPP_WARN(
        this->get_logger(), "Drop map saver cloud because PointCloud2 fields are invalid: stamp=%.6f",
        pending_cloud.stamp.seconds());
      return;
    }

    // cloudは全メッセージを候補として保持し、対応odometry到着後にscan bufferへ移す。
    pending_clouds_.push_back(std::move(pending_cloud));
    map_saved_since_last_update_ = false;
    match_pending_clouds();
  }

  void handle_odometry(const Odometry::SharedPtr message)
  {
    path_frame_ = message->header.frame_id;
    odometry_buffer_.push_back(StampedPose{
      rclcpp::Time(message->header.stamp), pose_to_transform(message->pose.pose)});
    prune_odometry_buffer(odometry_buffer_.back().stamp);
    match_pending_clouds();
  }

  void handle_path(const Path::SharedPtr message)
  {
    if (message->poses.empty()) {
      return;
    }

    // pathはloop closure後に更新されるため最新列を保存し、新規keyframeだけ初期poseとして記録する。
    map_saved_since_last_update_ = false;
    path_frame_ = message->header.frame_id;
    latest_path_stamp_ = rclcpp::Time(message->header.stamp);
    latest_path_ = message->poses;
    if (latest_path_.size() < initial_keyframe_poses_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received a shorter path; reset accumulated submaps.");
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
    next_submap_anchor_index_ = 0;
  }

  bool extract_cloud_points(const PointCloud2 & cloud, std::vector<CloudPoint> & points)
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

    // organized/unorganizedのどちらでも、PointCloud2のpoint_step単位でlocal点群だけを取り出す。
    const auto point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    points.clear();
    points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      const auto point_offset = index * cloud.point_step;
      CloudPoint point;
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
        scan_clouds_.push_back(ScanCloud{
          cloud_iter->stamp,
          odometry_buffer_[*odometry_index].path_from_lidar,
          std::move(cloud_iter->local_points)});
        cloud_iter = pending_clouds_.erase(cloud_iter);
        matched_any = true;
        continue;
      }
      if (!odometry_buffer_.empty() &&
        (odometry_buffer_.back().stamp - cloud_iter->stamp).seconds() > odometry_sync_tolerance_sec_)
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
        // cloud自体は受信できたが、対応するscan poseが無いためsubmap合成対象から外す。
        RCLCPP_WARN(
          this->get_logger(),
          "Drop map saver cloud because synchronized odometry was not found: cloud_stamp=%.6f best_odom_stamp=%.6f best_diff=%.6f tolerance=%.6f odom_oldest=%.6f odom_latest=%.6f pending_clouds=%zu points=%zu",
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
      std::sort(scan_clouds_.begin(), scan_clouds_.end(), [](const auto & lhs, const auto & rhs) {
        return (lhs.stamp - rhs.stamp).seconds() < 0.0;
      });
      finalize_ready_submaps();
    }
  }

  std::vector<CloudPoint> apply_voxel_filter(
    const std::vector<CloudPoint> & points, const double leaf_size) const
  {
    if (leaf_size <= 0.0) {
      return points;
    }

    // VoxelGrid相当の代表点抽出で、重なったscanの厚みだけを近傍leaf sizeの粒度へ抑える。
    std::unordered_map<VoxelKey, CloudPoint, VoxelKeyHash> voxel_points;
    voxel_points.reserve(points.size());
    for (const auto & point : points) {
      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(point.x / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.y / leaf_size)),
        static_cast<std::int64_t>(std::floor(point.z / leaf_size))};
      if (voxel_points.find(key) == voxel_points.end()) {
        voxel_points.emplace(key, point);
      }
    }

    std::vector<CloudPoint> filtered_points;
    filtered_points.reserve(voxel_points.size());
    for (const auto & [key, point] : voxel_points) {
      (void)key;
      filtered_points.push_back(point);
    }
    return filtered_points;
  }

  SubmapCloud create_submap(
    const std::size_t anchor_index, const std::optional<rclcpp::Time> & end_stamp) const
  {
    SubmapCloud submap;
    submap.anchor_index = anchor_index;
    submap.stamp = rclcpp::Time(initial_keyframe_poses_[anchor_index].header.stamp);
    const auto path_from_anchor_initial = pose_to_transform(initial_keyframe_poses_[anchor_index].pose);
    std::vector<CloudPoint> merged_points;

    // 各scanをkeyframe初期pose基準のlocal frameへ移し、区間内の全メッセージをsubmap化する。
    for (const auto & scan : scan_clouds_) {
      if ((scan.stamp - submap.stamp).seconds() < 0.0) {
        continue;
      }
      if (end_stamp.has_value() && (scan.stamp - *end_stamp).seconds() >= 0.0) {
        continue;
      }

      const auto anchor_from_scan = path_from_anchor_initial.inverse() * scan.path_from_lidar;
      ++submap.scan_count;
      merged_points.reserve(merged_points.size() + scan.local_points.size());
      for (const auto & local_point : scan.local_points) {
        const tf2::Vector3 transformed = anchor_from_scan *
          tf2::Vector3(local_point.x, local_point.y, local_point.z);
        merged_points.push_back(CloudPoint{
          static_cast<float>(transformed.x()),
          static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z()),
          local_point.intensity});
      }
    }

    submap.local_points = apply_voxel_filter(merged_points, submap_voxel_leaf_size_);
    return submap;
  }

  void erase_scans_before(const rclcpp::Time & stamp)
  {
    // 確定済みsubmapに合成したscanだけを破棄し、全期間のraw点群保持を避ける。
    scan_clouds_.erase(
      std::remove_if(
        scan_clouds_.begin(), scan_clouds_.end(),
        [&stamp](const ScanCloud & scan) { return (scan.stamp - stamp).seconds() < 0.0; }),
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
        "Finalized submap: anchor=%zu scans=%zu points=%zu",
        submap.anchor_index, submap.scan_count, submap.local_points.size());
      if (!submap.local_points.empty()) {
        submaps_.push_back(std::move(submap));
      }
      erase_scans_before(end_stamp);
      ++next_submap_anchor_index_;
    }
  }

  void save_map(const Trigger::Response::SharedPtr response)
  {
    response->success = save_map_file(response->message);
  }

  bool resolve_target_transform(tf2::Transform & transform, std::string & frame_id, std::string & message)
  {
    transform.setIdentity();
    frame_id = path_frame_;
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
    const SubmapCloud & submap,
    const tf2::Transform & target_from_path,
    std::vector<CloudPoint> & global_points) const
  {
    if (submap.anchor_index >= latest_path_.size()) {
      return;
    }

    // submapはanchor keyframe local frame保持なので、保存時は最新path poseでglobal配置だけを更新する。
    const auto path_from_anchor_latest = pose_to_transform(latest_path_[submap.anchor_index].pose);
    const auto target_from_anchor = target_from_path * path_from_anchor_latest;
    global_points.reserve(global_points.size() + submap.local_points.size());
    for (const auto & local_point : submap.local_points) {
      const tf2::Vector3 transformed = target_from_anchor *
        tf2::Vector3(local_point.x, local_point.y, local_point.z);
      global_points.push_back(CloudPoint{
        static_cast<float>(transformed.x()),
        static_cast<float>(transformed.y()),
        static_cast<float>(transformed.z()),
        local_point.intensity});
    }
  }

  std::vector<CloudPoint> build_global_points(const tf2::Transform & target_from_path) const
  {
    std::vector<CloudPoint> global_points;
    for (const auto & submap : submaps_) {
      append_submap_global_points(submap, target_from_path, global_points);
    }

    // 最後の未確定区間もサービス保存時には暫定submap化し、直近の全scanを出力へ含める。
    if (next_submap_anchor_index_ < initial_keyframe_poses_.size()) {
      const auto provisional_submap = create_submap(next_submap_anchor_index_, std::nullopt);
      append_submap_global_points(provisional_submap, target_from_path, global_points);
    }
    return global_points;
  }

  bool save_map_file(std::string & message)
  {
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

    auto global_points = apply_voxel_filter(build_global_points(target_from_path), global_voxel_leaf_size_);
    if (global_points.empty()) {
      message = "no accumulated scan cloud points to save";
      return false;
    }

    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {
      message = "failed to create output directory: " + error.message();
      return false;
    }

    // 最新path poseで再配置したhybrid submap群を標準的なASCII PCDとして保存する。
    const auto output_path = std::filesystem::path(output_directory_) /
      ("lio_sam_hybrid_map_" + timestamp_string() + ".pcd");
    std::ofstream file(output_path);
    if (!file) {
      message = "failed to open output file: " + output_path.string();
      return false;
    }

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z intensity\n";
    file << "SIZE 4 4 4 4\n";
    file << "TYPE F F F F\n";
    file << "COUNT 1 1 1 1\n";
    file << "WIDTH " << global_points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << global_points.size() << "\n";
    file << "DATA ascii\n";
    file << std::fixed << std::setprecision(6);
    for (const auto & point : global_points) {
      file << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << '\n';
    }

    message = output_path.string() + " frame=" + output_frame;
    map_saved_since_last_update_ = true;
    return true;
  }

  std::string target_frame_;
  std::string output_directory_;
  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string path_frame_;
  double odometry_sync_tolerance_sec_{};
  double cloud_buffer_duration_sec_{};
  double submap_voxel_leaf_size_{};
  double global_voxel_leaf_size_{};
  bool map_saved_since_last_update_{false};
  std::size_t next_submap_anchor_index_{};
  rclcpp::Time latest_path_stamp_{};
  std::vector<geometry_msgs::msg::PoseStamped> latest_path_;
  std::vector<geometry_msgs::msg::PoseStamped> initial_keyframe_poses_;
  std::vector<PendingCloud> pending_clouds_;
  std::vector<StampedPose> odometry_buffer_;
  std::vector<ScanCloud> scan_clouds_;
  std::vector<SubmapCloud> submaps_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<Path>::SharedPtr path_subscription_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::PcdMapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
