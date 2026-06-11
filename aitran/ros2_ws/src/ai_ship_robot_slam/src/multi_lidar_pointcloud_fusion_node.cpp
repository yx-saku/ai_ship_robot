#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ai_ship_robot_slam
{
namespace
{
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr auto kOutputPointStep = 32U;
constexpr auto kXOffset = 0U;
constexpr auto kYOffset = 4U;
constexpr auto kZOffset = 8U;
constexpr auto kIntensityOffset = 16U;
constexpr auto kRingOffset = 20U;
constexpr auto kTimeOffset = 24U;
constexpr auto kPi = 3.14159265358979323846;
constexpr auto kRadToDeg = 180.0 / kPi;
constexpr int kScanPatternAngleScale = 1000;
constexpr int kScanPatternFullCircleKey = 360 * kScanPatternAngleScale;

struct ScanPatternDirectionKey
{
  int azimuth_mdeg;
  int zenith_mdeg;

  bool operator==(const ScanPatternDirectionKey & other) const
  {
    return azimuth_mdeg == other.azimuth_mdeg && zenith_mdeg == other.zenith_mdeg;
  }
};

struct ScanPatternDirectionKeyHash
{
  std::size_t operator()(const ScanPatternDirectionKey & key) const
  {
    auto seed = std::hash<int>{}(key.azimuth_mdeg);
    seed ^= std::hash<int>{}(key.zenith_mdeg) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct CachedCustomMessage
{
  CustomMsg::SharedPtr message;
};

struct SyntheticPointInfo
{
  std::uint16_t ring;
};

template<typename T>
void write_unaligned(std::vector<std::uint8_t> & data, const std::size_t offset, const T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

PointField make_field(
  const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

double seconds_between(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return std::abs((lhs - rhs).seconds());
}

double normalize_degrees(const double degrees)
{
  auto normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

int normalize_azimuth_key(const int azimuth_mdeg)
{
  auto normalized = azimuth_mdeg % kScanPatternFullCircleKey;
  if (normalized < 0) {
    normalized += kScanPatternFullCircleKey;
  }
  return normalized;
}

ScanPatternDirectionKey make_scan_pattern_key(
  const double azimuth_deg, const double zenith_deg)
{
  return ScanPatternDirectionKey{
    normalize_azimuth_key(
      static_cast<int>(std::lround(normalize_degrees(azimuth_deg) * kScanPatternAngleScale))),
    static_cast<int>(std::lround(zenith_deg * kScanPatternAngleScale))};
}

tf2::Transform to_tf2_transform(const geometry_msgs::msg::Transform & transform)
{
  tf2::Quaternion rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  tf2::Vector3 translation(
    transform.translation.x, transform.translation.y, transform.translation.z);
  return tf2::Transform(tf2::Matrix3x3(rotation), translation);
}

std::string default_scan_pattern_csv_file()
{
  return ament_index_cpp::get_package_share_directory("ros2_livox_simulation") +
         "/scan_mode/mid360.csv";
}
}  // namespace

class MultiLidarPointCloudFusionNode : public rclcpp::Node
{
public:
  MultiLidarPointCloudFusionNode()
  : Node("multi_lidar_pointcloud_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 融合入力はCustomMsgを正とし、基準LiDARと出力topicはYAMLから受ける。
    input_custom_topics_ = declare_parameter<std::vector<std::string>>(
      "input_custom_topics", std::vector<std::string>{"/left_lidar/custom", "/right_lidar/custom"});
    output_points_topic_ = declare_parameter<std::string>(
      "output_points_topic", "/left_lidar/fused_points");
    reference_custom_topic_ = declare_parameter<std::string>(
      "reference_custom_topic", "/left_lidar/custom");
    reference_lidar_frame_ = declare_parameter<std::string>(
      "reference_lidar_frame", "left_lidar_link");
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    max_stamp_delta_sec_ = declare_parameter<double>("max_stamp_delta_sec", 0.05);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    timestamp_unit_scale_ = declare_parameter<double>("timestamp_unit_scale", 1.0e-9);
    max_relative_time_sec_ = declare_parameter<double>("max_relative_time_sec", 0.2);
    scan_pattern_csv_file_ = declare_parameter<std::string>(
      "scan_pattern_csv_file", default_scan_pattern_csv_file());
    synthesize_ring_from_pattern_ = declare_parameter<bool>("synthesize_ring_from_pattern", true);
    scan_pattern_physical_line_count_ = declare_parameter<int>("scan_pattern_physical_line_count", 4);
    synthetic_ring_count_ = declare_parameter<int>("synthetic_ring_count", 4);

    load_scan_pattern(scan_pattern_csv_file_);

    // LIO-SAM 前段の欠落を避けるため、CustomMsg入力とPointCloud2出力ともSensorDataQoSを使う。
    const auto qos = rclcpp::SensorDataQoS();
    fused_points_pub_ = create_publisher<PointCloud2>(output_points_topic_, qos);

    if (input_custom_topics_.empty()) {
      throw std::runtime_error("input_custom_topics must not be empty.");
    }
    if (
      std::find(input_custom_topics_.begin(), input_custom_topics_.end(), reference_custom_topic_) ==
      input_custom_topics_.end())
    {
      throw std::runtime_error("reference_custom_topic must be included in input_custom_topics.");
    }

    // 各LiDARの最新履歴を保持し、基準LiDAR到着時に近傍時刻のCustomMsgだけを融合する。
    subscriptions_.reserve(input_custom_topics_.size());
    for (const auto & topic : input_custom_topics_) {
      auto callback = [this, topic](const CustomMsg::SharedPtr message) {
          handle_custom(topic, std::move(message));
        };
      subscriptions_.push_back(create_subscription<CustomMsg>(topic, qos, callback));
    }

    RCLCPP_INFO(
      get_logger(),
      "Fuse CustomMsg for LIO-SAM: inputs=%zu reference_topic=%s reference_frame=%s output=%s pattern=%s",
      input_custom_topics_.size(), reference_custom_topic_.c_str(), reference_lidar_frame_.c_str(),
      output_points_topic_.c_str(), scan_pattern_csv_file_.c_str());
  }

private:
  void handle_custom(const std::string & topic, const CustomMsg::SharedPtr message)
  {
    {
      // 各topicの履歴を短く保持し、基準LiDAR到着時にheader.stampが最も近い候補を選べるようにする。
      std::scoped_lock lock(mutex_);
      auto & cache = cached_messages_[topic];
      cache.push_back(CachedCustomMessage{message});
      while (static_cast<int>(cache.size()) > std::max(1, sync_queue_size_)) {
        cache.pop_front();
      }
    }

    if (topic != reference_custom_topic_) {
      return;
    }

    PointCloud2 output;
    if (!build_fused_cloud(*message, output)) {
      return;
    }
    fused_points_pub_->publish(output);
  }

  bool build_fused_cloud(const CustomMsg & reference_cloud, PointCloud2 & output)
  {
    const rclcpp::Time reference_stamp(reference_cloud.header.stamp);
    std::vector<PointCloud2> transformed_clouds;
    transformed_clouds.reserve(input_custom_topics_.size());

    {
      // 基準LiDAR時刻に最も近いCustomMsgを各topicから選び、使えるものだけを今回の合成対象にする。
      std::scoped_lock lock(mutex_);
      for (const auto & topic : input_custom_topics_) {
        const auto cached_cloud = find_best_matching_cloud(topic, reference_stamp);
        if (!cached_cloud.has_value()) {
          if (topic != reference_custom_topic_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Skip topic %s because no cached CustomMsg is close enough to reference stamp.",
              topic.c_str());
            continue;
          }

          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skip fusion because reference topic %s has no cached CustomMsg.", topic.c_str());
          return false;
        }

        PointCloud2 transformed_cloud;
        if (!convert_custom_to_cloud(cached_cloud.value(), reference_stamp, transformed_cloud)) {
          if (topic == reference_custom_topic_) {
            return false;
          }

          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skip topic %s because CustomMsg transform into reference frame failed.",
            topic.c_str());
          continue;
        }
        transformed_clouds.push_back(std::move(transformed_cloud));
      }
    }

    if (transformed_clouds.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip fusion because no CustomMsg was available for publication.");
      return false;
    }

    // 融合後もLIO-SAM互換のfield layoutを維持するため、固定レイアウトの生バイト列を連結する。
    output = transformed_clouds.front();
    std::size_t total_points = 0;
    for (const auto & cloud : transformed_clouds) {
      total_points += static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
    }

    output.header.stamp = reference_cloud.header.stamp;
    output.header.frame_id = reference_lidar_frame_;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(total_points);
    output.row_step = output.point_step * output.width;
    output.data.clear();
    output.data.reserve(total_points * output.point_step);
    output.is_dense = true;

    for (const auto & cloud : transformed_clouds) {
      output.is_dense = output.is_dense && cloud.is_dense;
      output.data.insert(output.data.end(), cloud.data.begin(), cloud.data.end());
    }

    return true;
  }

  std::optional<CachedCustomMessage> find_best_matching_cloud(
    const std::string & topic, const rclcpp::Time & reference_stamp) const
  {
    const auto iterator = cached_messages_.find(topic);
    if (iterator == cached_messages_.end() || iterator->second.empty()) {
      return std::nullopt;
    }

    CachedCustomMessage best_cloud{};
    auto found = false;
    auto best_delta = std::numeric_limits<double>::max();
    for (const auto & candidate : iterator->second) {
      if (candidate.message == nullptr) {
        continue;
      }

      const auto delta = seconds_between(reference_stamp, rclcpp::Time(candidate.message->header.stamp));
      if (delta < best_delta) {
        best_delta = delta;
        best_cloud = candidate;
        found = true;
      }
    }

    // 時刻差が大きい点群は採用せず、今回の合成に使えるLiDARだけでpublishする。
    if (!found || best_delta > max_stamp_delta_sec_) {
      return std::nullopt;
    }
    return best_cloud;
  }

  bool convert_custom_to_cloud(
    const CachedCustomMessage & cached_input, const rclcpp::Time & reference_stamp, PointCloud2 & output)
  {
    const auto & input = *cached_input.message;
    tf2::Transform transform;
    if (input.header.frame_id == reference_lidar_frame_) {
      transform.setIdentity();
    } else {
      try {
        // LiDARごとの固定TFを使って基準LiDAR座標系へ正規化し、LIO-SAM入力座標系を統一する。
        const auto stamped_transform = tf_buffer_.lookupTransform(
          reference_lidar_frame_, input.header.frame_id, input.header.stamp,
          tf2::durationFromSec(tf_timeout_sec_));
        transform = to_tf2_transform(stamped_transform.transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform CustomMsg from %s to %s: %s", input.header.frame_id.c_str(),
          reference_lidar_frame_.c_str(), ex.what());
        return false;
      }
    }

    // LIO-SAM互換PointCloud2へ変換し、simulation前提のtimeは後段で0固定にする。
    output.header = input.header;
    output.header.frame_id = reference_lidar_frame_;
    output.header.stamp = reference_stamp;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(input.points.size());
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = kOutputPointStep;
    output.row_step = output.point_step * output.width;
    output.fields = {
      make_field("x", kXOffset, PointField::FLOAT32),
      make_field("y", kYOffset, PointField::FLOAT32),
      make_field("z", kZOffset, PointField::FLOAT32),
      make_field("intensity", kIntensityOffset, PointField::FLOAT32),
      make_field("ring", kRingOffset, PointField::UINT16),
      make_field("time", kTimeOffset, PointField::FLOAT32),
    };
    output.data.resize(static_cast<std::size_t>(output.row_step));

    const auto source_stamp = rclcpp::Time(input.header.stamp);
    const auto delta_sec = (source_stamp - reference_stamp).seconds();
    const auto synthetic_info = build_synthetic_point_info(input);
    for (std::uint32_t output_index = 0; output_index < output.width; ++output_index) {
      output.is_dense = output.is_dense && append_point(
        input.points[output_index], transform, delta_sec, synthetic_info[output_index], output,
        output_index);
    }
    return true;
  }

  std::vector<SyntheticPointInfo> build_synthetic_point_info(const CustomMsg & input) const
  {
    std::vector<SyntheticPointInfo> info(input.points.size());

    // Livox pluginがlineを出さないsimulation点群では、CSVの走査方向から本来のlineを復元する。
    for (std::size_t index = 0; index < input.points.size(); ++index) {
      const auto & point = input.points[index];
      if (synthesize_ring_from_pattern_ && point.line == 0U && synthetic_ring_count_ > 1) {
        info[index].ring = lookup_scan_pattern_ring(point).value_or(fallback_ring(index));
      } else {
        info[index].ring = static_cast<std::uint16_t>(point.line);
      }
    }
    return info;
  }

  std::uint16_t fallback_ring(const std::size_t point_index) const
  {
    return static_cast<std::uint16_t>(
      point_index % static_cast<std::size_t>(std::max(1, synthetic_ring_count_)));
  }

  std::optional<std::uint16_t> lookup_scan_pattern_ring(const CustomPoint & point) const
  {
    const auto maybe_key = make_point_direction_key(point);
    if (!maybe_key.has_value()) {
      return std::nullopt;
    }

    // 点群座標はfloat経由で丸められるため、ミリ度keyの隣接セルまで許容して照合する。
    const auto & key = maybe_key.value();
    for (int zenith_delta = -1; zenith_delta <= 1; ++zenith_delta) {
      for (int azimuth_delta = -1; azimuth_delta <= 1; ++azimuth_delta) {
        const auto candidate = ScanPatternDirectionKey{
          normalize_azimuth_key(key.azimuth_mdeg + azimuth_delta),
          key.zenith_mdeg + zenith_delta};
        const auto match = scan_pattern_line_by_direction_.find(candidate);
        if (match != scan_pattern_line_by_direction_.end()) {
          return match->second;
        }
      }
    }
    return std::nullopt;
  }

  std::optional<ScanPatternDirectionKey> make_point_direction_key(const CustomPoint & point) const
  {
    const auto horizontal_range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    const auto range = std::hypot(horizontal_range, static_cast<double>(point.z));
    if (range <= std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }

    // pluginのray生成式と逆変換し、点の方向をCSVのAzimuth/Zenith表現へ戻す。
    const auto azimuth_deg = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) *
      kRadToDeg;
    const auto zenith_deg = 90.0 -
      std::atan2(static_cast<double>(point.z), horizontal_range) * kRadToDeg;
    return make_scan_pattern_key(azimuth_deg, zenith_deg);
  }

  void load_scan_pattern(const std::string & csv_file)
  {
    std::ifstream stream(csv_file);
    std::string line;
    scan_pattern_line_by_direction_.clear();

    if (!stream.is_open()) {
      throw std::runtime_error("Failed to open scan pattern csv: " + csv_file);
    }

    // CSVの行順がLivox scan patternの照射順なので、行番号からlineを事前計算してlookup化する。
    if (!std::getline(stream, line)) {
      throw std::runtime_error("Scan pattern csv is empty: " + csv_file);
    }
    std::size_t pattern_index = 0;
    std::size_t direction_collision_count = 0;
    while (std::getline(stream, line)) {
      if (line.empty()) {
        continue;
      }
      std::stringstream line_stream(line);
      std::string time_value;
      std::string azimuth_value;
      std::string zenith_value;
      if (
        !std::getline(line_stream, time_value, ',') ||
        !std::getline(line_stream, azimuth_value, ',') ||
        !std::getline(line_stream, zenith_value, ','))
      {
        continue;
      }

      try {
        const auto key = make_scan_pattern_key(std::stod(azimuth_value), std::stod(zenith_value));
        const auto physical_line_count = static_cast<std::size_t>(
          std::max(1, scan_pattern_physical_line_count_));
        const auto synthetic_ring_count = static_cast<std::size_t>(std::max(1, synthetic_ring_count_));
        // 物理lineは有効点が偏るため、line内の進行順をLIO-SAM用の仮想ringへ均等に割り当てる。
        const auto ring = static_cast<std::uint16_t>(
          (pattern_index / physical_line_count) % synthetic_ring_count);
        const auto [iterator, inserted] = scan_pattern_line_by_direction_.emplace(key, ring);
        if (!inserted && iterator->second != ring) {
          ++direction_collision_count;
        }
      } catch (const std::exception &) {
        continue;
      }
      ++pattern_index;
    }

    if (scan_pattern_line_by_direction_.empty()) {
      throw std::runtime_error("Scan pattern csv must contain at least two rows: " + csv_file);
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu Livox scan pattern directions from %s.",
      scan_pattern_line_by_direction_.size(), csv_file.c_str());
    if (direction_collision_count > 0U) {
      RCLCPP_WARN(
        get_logger(), "Livox scan pattern had %zu direction key collisions with different lines.",
        direction_collision_count);
    }
  }

  bool append_point(
    const CustomPoint & point, const tf2::Transform & transform, const double /*delta_sec*/,
    const SyntheticPointInfo & synthetic_info, PointCloud2 & output,
    const std::uint32_t output_index) const
  {
    const tf2::Vector3 source_point(point.x, point.y, point.z);
    const tf2::Vector3 transformed_point = transform * source_point;
    if (
      !std::isfinite(transformed_point.x()) || !std::isfinite(transformed_point.y()) ||
      !std::isfinite(transformed_point.z()))
    {
      return false;
    }

    // simulation点群はscan内で同一姿勢のため、LIO-SAMへは全点time=0を渡してdeskewを実質無効化する。
    const auto output_offset = static_cast<std::size_t>(output_index) * kOutputPointStep;
    write_unaligned<float>(output.data, output_offset + kXOffset, static_cast<float>(transformed_point.x()));
    write_unaligned<float>(output.data, output_offset + kYOffset, static_cast<float>(transformed_point.y()));
    write_unaligned<float>(output.data, output_offset + kZOffset, static_cast<float>(transformed_point.z()));
    write_unaligned<float>(
      output.data, output_offset + kIntensityOffset, static_cast<float>(point.reflectivity));
    write_unaligned<std::uint16_t>(output.data, output_offset + kRingOffset, synthetic_info.ring);
    write_unaligned<float>(output.data, output_offset + kTimeOffset, 0.0F);
    return true;
  }

  std::vector<std::string> input_custom_topics_;
  std::string output_points_topic_;
  std::string reference_custom_topic_;
  std::string reference_lidar_frame_;
  int sync_queue_size_;
  double max_stamp_delta_sec_;
  double tf_timeout_sec_;
  double timestamp_unit_scale_;
  double max_relative_time_sec_;
  std::string scan_pattern_csv_file_;
  bool synthesize_ring_from_pattern_;
  int scan_pattern_physical_line_count_;
  int synthetic_ring_count_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::deque<CachedCustomMessage>> cached_messages_;
  std::unordered_map<
    ScanPatternDirectionKey, std::uint16_t, ScanPatternDirectionKeyHash>
    scan_pattern_line_by_direction_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<rclcpp::Subscription<CustomMsg>::SharedPtr> subscriptions_;
  rclcpp::Publisher<PointCloud2>::SharedPtr fused_points_pub_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::MultiLidarPointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
