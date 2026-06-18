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
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstdint>

#include <algorithm>
#include <deque>
#include <fstream>
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
#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ai_ship_robot_slam
{
namespace
{
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

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

struct MatchedCustomMessage
{
  std::string topic;
  CustomMsg::SharedPtr message;
};

struct SyntheticPointInfo
{
  std::uint16_t ring;
};

struct CachedRigidTransform
{
  bool identity{false};
  float r00{1.0F};
  float r01{0.0F};
  float r02{0.0F};
  float r10{0.0F};
  float r11{1.0F};
  float r12{0.0F};
  float r20{0.0F};
  float r21{0.0F};
  float r22{1.0F};
  float tx{0.0F};
  float ty{0.0F};
  float tz{0.0F};
};

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

std::string default_scan_pattern_csv_file()
{
  return ament_index_cpp::get_package_share_directory("ros2_livox_simulation") +
         "/scan_mode/mid360.csv";
}

double point_offset_seconds(const CustomPoint & point, const double timestamp_unit_scale)
{
  // Livox CustomMsgのoffset_time単位を秒へそろえ、無効なscaleではdeskew用時刻を落とす。
  if (timestamp_unit_scale <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(point.offset_time) * timestamp_unit_scale;
}

std::uint32_t seconds_to_offset_time(
  const double seconds, const double timestamp_unit_scale, const double max_relative_time_sec)
{
  // 秒へ補正済みの相対時刻をCustomMsgの整数offset_timeへ戻し、範囲外は安全側に丸める。
  if (timestamp_unit_scale <= 0.0) {
    return 0U;
  }

  const auto max_relative_time = std::max(0.0, max_relative_time_sec);
  const auto bounded_seconds = std::clamp(seconds, 0.0, max_relative_time);
  const auto raw_offset = bounded_seconds / timestamp_unit_scale;
  if (raw_offset <= 0.0) {
    return 0U;
  }
  if (raw_offset >= static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return static_cast<std::uint32_t>(std::llround(raw_offset));
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
    // 融合入力はCustomMsgに限定し、PointCloud2出力は作らずLIO-SAM入力topicだけへpublishする。
    input_custom_topics_ = declare_parameter<std::vector<std::string>>(
      "input_custom_topics", std::vector<std::string>{"/lidar1/livox/lidar"});
    output_custom_topic_ = declare_parameter<std::string>(
      "output_custom_topic", "/fused/livox/lidar");
    reference_custom_topic_ = declare_parameter<std::string>(
      "reference_custom_topic", "/lidar1/livox/lidar");
    input_ring_offsets_ = declare_parameter<std::vector<int64_t>>(
      "input_ring_offsets", std::vector<int64_t>{});
    cache_size_limit_ = declare_parameter<int>("cache_size_limit", 0);
    max_stamp_delta_sec_ = declare_parameter<double>("max_stamp_delta_sec", 0.05);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    timestamp_unit_scale_ = declare_parameter<double>("timestamp_unit_scale", 1.0e-9);
    max_relative_time_sec_ = declare_parameter<double>("max_relative_time_sec", 0.2);
    scan_pattern_csv_file_ = declare_parameter<std::string>(
      "scan_pattern_csv_file", default_scan_pattern_csv_file());
    synthesize_ring_from_pattern_ = declare_parameter<bool>("synthesize_ring_from_pattern", false);
    scan_pattern_physical_line_count_ = declare_parameter<int>(
      "scan_pattern_physical_line_count", 4);
    synthetic_ring_count_ = declare_parameter<int>("synthetic_ring_count", 4);

    load_scan_pattern(scan_pattern_csv_file_);

    // LIO-SAM前段の欠落を避けるため、CustomMsg入出力はSensorDataQoSでそろえる。
    const auto qos = rclcpp::SensorDataQoS();
    fused_custom_pub_ = create_publisher<CustomMsg>(output_custom_topic_, qos);

    if (input_custom_topics_.empty()) {
      throw std::runtime_error("input_custom_topics must not be empty.");
    }
    if (
      std::find(
        input_custom_topics_.begin(), input_custom_topics_.end(),
        reference_custom_topic_) ==
      input_custom_topics_.end())
    {
      throw std::runtime_error("reference_custom_topic must be included in input_custom_topics.");
    }
    if (
      std::find(input_custom_topics_.begin(), input_custom_topics_.end(), output_custom_topic_) !=
      input_custom_topics_.end())
    {
      throw std::runtime_error("output_custom_topic must not overlap with input_custom_topics.");
    }
    if (!input_ring_offsets_.empty() && input_ring_offsets_.size() != input_custom_topics_.size()) {
      throw std::runtime_error(
              "input_ring_offsets must be empty or have the same size as input_custom_topics.");
    }

    // 各LiDARの未処理履歴を保持し、基準scanが2件そろった時点で最古の基準scanを確定させる。
    subscriptions_.reserve(input_custom_topics_.size());
    for (const auto & topic : input_custom_topics_) {
      auto callback = [this, topic](const CustomMsg::SharedPtr message) {
          handle_custom(topic, message);
        };
      subscriptions_.push_back(create_subscription<CustomMsg>(topic, qos, callback));
    }

    RCLCPP_INFO(
      get_logger(),
      "Fuse CustomMsg for LIO-SAM: inputs=%zu reference_topic=%s custom_output=%s pattern=%s",
      input_custom_topics_.size(), reference_custom_topic_.c_str(), output_custom_topic_.c_str(),
      scan_pattern_csv_file_.c_str());
  }

private:
  void handle_custom(const std::string & topic, const CustomMsg::SharedPtr & message)
  {
    bool should_process_reference = false;
    {
      // topicごとの履歴は到着順に積み、基準topicだけ2件目以降で処理を進められるようにする。
      std::scoped_lock lock(mutex_);
      auto & queue = cached_messages_[topic];
      queue.push_back(message);
      enforce_cache_limit(queue);
      if (topic == reference_custom_topic_ && queue.size() >= 2U) {
        should_process_reference = true;
      }
    }

    if (!should_process_reference) {
      return;
    }

    while (process_oldest_reference_scan()) {
    }
  }

  bool process_oldest_reference_scan()
  {
    CustomMsg::SharedPtr reference_message;
    {
      // 次の基準scanが到着した後にだけ最古scanを確定し、後続LiDARの遅延到着を吸収する。
      std::scoped_lock lock(mutex_);
      const auto iterator = cached_messages_.find(reference_custom_topic_);
      if (iterator == cached_messages_.end() || iterator->second.size() < 2U) {
        return false;
      }
      reference_message = iterator->second.front();
    }

    if (reference_message == nullptr) {
      return false;
    }

    CustomMsg output_custom;
    if (!build_fused_custom(*reference_message, output_custom)) {
      prune_processed_messages(rclcpp::Time(reference_message->header.stamp));
      return true;
    }
    fused_custom_pub_->publish(output_custom);
    prune_processed_messages(rclcpp::Time(reference_message->header.stamp));
    return true;
  }

  bool build_fused_custom(const CustomMsg & reference_cloud, CustomMsg & output_custom)
  {
    const rclcpp::Time reference_stamp(reference_cloud.header.stamp);
    const auto reference_frame = reference_cloud.header.frame_id;
    if (reference_frame.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip fusion because reference topic %s has an empty frame_id.",
        reference_custom_topic_.c_str());
      return false;
    }

    std::vector<MatchedCustomMessage> matched_messages;
    matched_messages.reserve(input_custom_topics_.size());

    {
      // 基準scanに対して各topicの最近傍scanだけを選び、使えないLiDARは今回分から外す。
      std::scoped_lock lock(mutex_);
      for (const auto & topic : input_custom_topics_) {
        const auto matched_message = find_best_matching_cloud(topic, reference_stamp);
        if (!matched_message.has_value()) {
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
        matched_messages.push_back(MatchedCustomMessage{topic, matched_message.value()});
      }
    }

    if (matched_messages.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip fusion because no CustomMsg was available for publication.");
      return false;
    }

    std::size_t total_points = 0;
    for (const auto & matched_message : matched_messages) {
      if (matched_message.message != nullptr) {
        total_points += matched_message.message->points.size();
      }
    }

    // 出力はCustomMsgだけに絞り、最終vectorへ直接書き込んで中間コピーを避ける。
    output_custom = reference_cloud;
    output_custom.header.stamp = reference_cloud.header.stamp;
    output_custom.header.frame_id = reference_frame;
    output_custom.points.resize(total_points);
    output_custom.point_num = static_cast<std::uint32_t>(total_points);

    std::size_t output_index = 0U;
    for (const auto & matched_message : matched_messages) {
      if (matched_message.message == nullptr) {
        continue;
      }

      const auto transform = get_cached_transform(
        reference_frame, matched_message.message->header.frame_id,
        matched_message.message->header.stamp);
      if (!transform.has_value()) {
        if (matched_message.topic == reference_custom_topic_) {
          return false;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skip topic frame %s because transform into reference frame %s failed.",
          matched_message.message->header.frame_id.c_str(), reference_frame.c_str());
        continue;
      }

      const auto synthetic_info = build_synthetic_point_info(
        *matched_message.message, ring_offset_for_topic(matched_message.topic));
      const auto message_stamp = rclcpp::Time(matched_message.message->header.stamp);
      const auto message_delta_sec = (message_stamp - reference_stamp).seconds();
      for (std::size_t point_index = 0; point_index < matched_message.message->points.size();
        ++point_index)
      {
        // 各点は基準frameへ直接変換し、header差分を点ごとの相対時刻へ反映する。
        const auto appended = append_transformed_point(
          matched_message.message->points[point_index], synthetic_info[point_index],
          transform.value(), message_delta_sec, output_custom.points[output_index]);
        if (appended) {
          ++output_index;
        }
      }
    }

    output_custom.points.resize(output_index);
    output_custom.point_num = static_cast<std::uint32_t>(output_custom.points.size());
    if (output_custom.points.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip fusion because all candidate point clouds were filtered out.");
      return false;
    }

    return true;
  }

  std::optional<CustomMsg::SharedPtr> find_best_matching_cloud(
    const std::string & topic, const rclcpp::Time & reference_stamp) const
  {
    const auto iterator = cached_messages_.find(topic);
    if (iterator == cached_messages_.end() || iterator->second.empty()) {
      return std::nullopt;
    }

    CustomMsg::SharedPtr best_message;
    auto best_delta = std::numeric_limits<double>::max();
    for (const auto & candidate : iterator->second) {
      if (candidate == nullptr) {
        continue;
      }

      const auto delta = seconds_between(reference_stamp, rclcpp::Time(candidate->header.stamp));
      if (delta < best_delta) {
        best_delta = delta;
        best_message = candidate;
      }
    }

    // 時刻差が大きい点群は採用せず、今回の合成に使えるLiDARだけでpublishする。
    if (best_message == nullptr || best_delta > max_stamp_delta_sec_) {
      return std::nullopt;
    }
    return best_message;
  }

  std::optional<CachedRigidTransform> get_cached_transform(
    const std::string & target_frame, const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (target_frame == source_frame) {
      CachedRigidTransform identity_transform;
      identity_transform.identity = true;
      return identity_transform;
    }

    const auto cache_key = target_frame + "<-" + source_frame;
    {
      // LiDAR間TFは固定として扱い、既に展開済みの回転行列と並進を再利用する。
      std::scoped_lock lock(transform_mutex_);
      const auto iterator = transform_cache_.find(cache_key);
      if (iterator != transform_cache_.end()) {
        return iterator->second;
      }
    }

    try {
      // 初回だけtf2から取得し、点ごとの変換ではtf2::Transform生成と演算を避ける。
      const auto stamped_transform = tf_buffer_.lookupTransform(
        target_frame, source_frame, stamp, tf2::durationFromSec(tf_timeout_sec_));
      const auto & translation = stamped_transform.transform.translation;
      const auto & rotation = stamped_transform.transform.rotation;
      tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
      tf2::Matrix3x3 matrix(quaternion);

      CachedRigidTransform cached_transform;
      cached_transform.r00 = static_cast<float>(matrix[0][0]);
      cached_transform.r01 = static_cast<float>(matrix[0][1]);
      cached_transform.r02 = static_cast<float>(matrix[0][2]);
      cached_transform.r10 = static_cast<float>(matrix[1][0]);
      cached_transform.r11 = static_cast<float>(matrix[1][1]);
      cached_transform.r12 = static_cast<float>(matrix[1][2]);
      cached_transform.r20 = static_cast<float>(matrix[2][0]);
      cached_transform.r21 = static_cast<float>(matrix[2][1]);
      cached_transform.r22 = static_cast<float>(matrix[2][2]);
      cached_transform.tx = static_cast<float>(translation.x);
      cached_transform.ty = static_cast<float>(translation.y);
      cached_transform.tz = static_cast<float>(translation.z);

      std::scoped_lock lock(transform_mutex_);
      transform_cache_[cache_key] = cached_transform;
      return cached_transform;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform CustomMsg from %s to %s: %s", source_frame.c_str(),
        target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  int64_t ring_offset_for_topic(const std::string & topic) const
  {
    const auto iterator =
      std::find(input_custom_topics_.begin(), input_custom_topics_.end(), topic);
    if (iterator == input_custom_topics_.end() || input_ring_offsets_.empty()) {
      return 0;
    }
    return input_ring_offsets_[static_cast<std::size_t>(iterator - input_custom_topics_.begin())];
  }

  std::uint16_t apply_ring_offset(const std::uint16_t ring, const int64_t ring_offset) const
  {
    const auto shifted_ring = static_cast<int64_t>(ring) + ring_offset;
    if (shifted_ring < 0) {
      return 0U;
    }
    return static_cast<std::uint16_t>(std::min<int64_t>(
             shifted_ring, std::numeric_limits<std::uint16_t>::max()));
  }

  std::vector<SyntheticPointInfo> build_synthetic_point_info(
    const CustomMsg & input, const int64_t ring_offset) const
  {
    std::vector<SyntheticPointInfo> info(input.points.size());

    // LiDARごとにring帯域を分け、fusion後もLIO-SAMのrange image上で各LiDARを独立行へ載せる。
    for (std::size_t index = 0; index < input.points.size(); ++index) {
      const auto & point = input.points[index];
      std::uint16_t ring = 0U;
      if (synthesize_ring_from_pattern_ && point.line == 0U && synthetic_ring_count_ > 1) {
        ring = lookup_scan_pattern_ring(point).value_or(fallback_ring(index));
      } else {
        ring = static_cast<std::uint16_t>(point.line);
      }
      info[index].ring = apply_ring_offset(ring, ring_offset);
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
    const auto horizontal_range = std::hypot(
      static_cast<double>(point.x), static_cast<double>(point.y));
    const auto range = std::hypot(horizontal_range, static_cast<double>(point.z));
    if (range <= std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }

    // pluginのray生成式と逆変換し、点の方向をCSVのAzimuth/Zenith表現へ戻す。
    const auto azimuth_deg =
      std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) *
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
        const auto synthetic_ring_count = static_cast<std::size_t>(std::max(
            1,
            synthetic_ring_count_));
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

  bool append_transformed_point(
    const CustomPoint & point, const SyntheticPointInfo & synthetic_info,
    const CachedRigidTransform & transform, const double message_delta_sec,
    CustomPoint & output_point) const
  {
    const auto transformed_x = transform.identity ? point.x :
      transform.r00 * point.x + transform.r01 * point.y + transform.r02 * point.z + transform.tx;
    const auto transformed_y = transform.identity ? point.y :
      transform.r10 * point.x + transform.r11 * point.y + transform.r12 * point.z + transform.ty;
    const auto transformed_z = transform.identity ? point.z :
      transform.r20 * point.x + transform.r21 * point.y + transform.r22 * point.z + transform.tz;
    if (
      !std::isfinite(transformed_x) || !std::isfinite(transformed_y) ||
      !std::isfinite(transformed_z))
    {
      return false;
    }

    // 異なるscan時刻の点群を基準scanの相対時刻系へ揃え、LIO-SAM用CustomMsgへ直接反映する。
    const auto adjusted_offset_sec = message_delta_sec +
      point_offset_seconds(point, timestamp_unit_scale_);
    output_point = point;
    output_point.offset_time = seconds_to_offset_time(
      adjusted_offset_sec, timestamp_unit_scale_, max_relative_time_sec_);
    output_point.x = transformed_x;
    output_point.y = transformed_y;
    output_point.z = transformed_z;
    output_point.line = static_cast<std::uint8_t>(std::min<std::uint16_t>(
        synthetic_info.ring, std::numeric_limits<std::uint8_t>::max()));
    return true;
  }

  void prune_processed_messages(const rclcpp::Time & processed_reference_stamp)
  {
    // 処理済み基準scanを落とし、非基準topicは次回最近傍探索に必要な境界候補を残して剪定する。
    std::scoped_lock lock(mutex_);
    const auto reference_iterator = cached_messages_.find(reference_custom_topic_);
    if (reference_iterator != cached_messages_.end() && !reference_iterator->second.empty()) {
      reference_iterator->second.pop_front();
    }

    for (auto & [topic, queue] : cached_messages_) {
      if (topic == reference_custom_topic_) {
        continue;
      }
      while (queue.size() >= 2U) {
        if (queue[1] == nullptr) {
          queue.pop_front();
          continue;
        }
        const auto next_stamp = rclcpp::Time(queue[1]->header.stamp);
        if (next_stamp <= processed_reference_stamp) {
          queue.pop_front();
          continue;
        }
        break;
      }
      enforce_cache_limit(queue);
    }
  }

  void enforce_cache_limit(std::deque<CustomMsg::SharedPtr> & queue) const
  {
    if (cache_size_limit_ <= 0) {
      return;
    }
    while (static_cast<int>(queue.size()) > cache_size_limit_) {
      queue.pop_front();
    }
  }

  std::vector<std::string> input_custom_topics_;
  std::string output_custom_topic_;
  std::string reference_custom_topic_;
  std::vector<int64_t> input_ring_offsets_;
  int cache_size_limit_;
  double max_stamp_delta_sec_;
  double tf_timeout_sec_;
  double timestamp_unit_scale_;
  double max_relative_time_sec_;
  std::string scan_pattern_csv_file_;
  bool synthesize_ring_from_pattern_;
  int scan_pattern_physical_line_count_;
  int synthetic_ring_count_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::deque<CustomMsg::SharedPtr>> cached_messages_;
  std::mutex transform_mutex_;
  std::unordered_map<std::string, CachedRigidTransform> transform_cache_;
  std::unordered_map<
    ScanPatternDirectionKey, std::uint16_t, ScanPatternDirectionKeyHash>
  scan_pattern_line_by_direction_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<rclcpp::Subscription<CustomMsg>::SharedPtr> subscriptions_;
  rclcpp::Publisher<CustomMsg>::SharedPtr fused_custom_pub_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::MultiLidarPointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
