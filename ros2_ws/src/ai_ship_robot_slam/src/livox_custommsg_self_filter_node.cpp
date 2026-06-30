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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
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

tf2::Transform transform_from_message(const geometry_msgs::msg::Transform & message)
{
  tf2::Quaternion rotation(
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w);
  if (!is_finite_quaternion(rotation)) {
    throw std::runtime_error("self filter transform has an invalid rotation quaternion");
  }
  rotation.normalize();

  const tf2::Vector3 translation(
    message.translation.x, message.translation.y, message.translation.z);
  if (!is_finite(translation.x()) || !is_finite(translation.y()) || !is_finite(translation.z())) {
    throw std::runtime_error("self filter transform has an invalid translation");
  }
  return tf2::Transform(rotation, translation);
}

}  // namespace

class LivoxCustomMsgSelfFilterNode : public rclcpp::Node
{
public:
  explicit LivoxCustomMsgSelfFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("livox_custommsg_self_filter_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    // override済みなら既存値を優先し、未指定時だけ既定値を宣言して使う。
    input_topics_ = get_or_declare_parameter<std::vector<std::string>>(
      "input_topics", std::vector<std::string>{"/lidar1/livox/lidar", "/lidar2/livox/lidar"});
    output_topics_ = get_or_declare_parameter<std::vector<std::string>>(
      "output_topics",
      std::vector<std::string>{"/lidar1/livox/lidar_filtered", "/lidar2/livox/lidar_filtered"});
    filter_frame_ = get_or_declare_parameter<std::string>("filter_frame", "base_footprint");
    margin_ = get_or_declare_parameter<double>("margin", 0.03);
    tf_timeout_sec_ = get_or_declare_parameter<double>("tf_timeout_sec", 0.1);
    drop_on_tf_failure_ = get_or_declare_parameter<bool>("drop_on_tf_failure", true);

    validate_common_parameters();
    boxes_ = load_boxes();
    create_channels();

    if (boxes_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No self filter boxes are configured. CustomMsg will be forwarded without point removal.");
    }
    RCLCPP_INFO(
      get_logger(), "Livox self filter started: frame=%s boxes=%zu margin=%.3f",
      filter_frame_.c_str(), boxes_.size(), margin_);
  }

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;

  template<typename T>
  T get_or_declare_parameter(const std::string & name, const T & default_value)
  {
    T value{};
    if (this->has_parameter(name) && this->get_parameter(name, value)) {
      return value;
    }
    return this->declare_parameter<T>(name, default_value);
  }

  struct Range
  {
    double min{0.0};
    double max{0.0};
  };

  struct Box
  {
    Range x;
    Range y;
    Range z;
  };

  struct Channel
  {
    std::string output_topic;
    std::string lidar_frame;
    std::optional<tf2::Transform> filter_from_lidar;
    rclcpp::Publisher<CustomMsg>::SharedPtr publisher;
    rclcpp::Subscription<CustomMsg>::SharedPtr subscription;
  };

  void validate_common_parameters() const
  {
    if (input_topics_.empty()) {
      throw std::invalid_argument("input_topics must contain at least one topic");
    }
    if (input_topics_.size() != output_topics_.size()) {
      throw std::invalid_argument("input_topics and output_topics must have the same length");
    }
    if (filter_frame_.empty()) {
      throw std::invalid_argument("filter_frame must not be empty");
    }
    if (!std::isfinite(margin_) || margin_ < 0.0) {
      throw std::invalid_argument("margin must be a finite non-negative value");
    }
    if (!std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ < 0.0) {
      throw std::invalid_argument("tf_timeout_sec must be a finite non-negative value");
    }
    // topicの空文字や重複は誤配線に直結するため、起動時に明示的に止める。
    for (std::size_t i = 0; i < input_topics_.size(); ++i) {
      if (input_topics_[i].empty() || output_topics_[i].empty()) {
        throw std::invalid_argument("input/output topics must not contain an empty topic");
      }
      if (input_topics_[i] == output_topics_[i]) {
        throw std::invalid_argument("input topic and output topic must be different");
      }
      for (std::size_t j = i + 1; j < input_topics_.size(); ++j) {
        if (input_topics_[i] == input_topics_[j] || output_topics_[i] == output_topics_[j]) {
          throw std::invalid_argument("input_topics and output_topics must not contain duplicates");
        }
      }
    }
  }

  std::vector<Box> load_boxes()
  {
    constexpr std::size_t values_per_box = 6;
    std::vector<std::pair<std::string, std::vector<double>>> named_values;
    const auto listed = this->list_parameters({"boxes"}, 2U);

    // YAMLのboxesネストを自動宣言済みパラメータとして受け取り、子要素だけを抽出する。
    for (const auto & parameter_name : listed.names) {
      if (parameter_name.rfind("boxes.", 0) != 0) {
        continue;
      }

      rclcpp::Parameter parameter;
      if (!this->get_parameter(parameter_name, parameter)) {
        continue;
      }

      const auto box_name = parameter_name.substr(std::string("boxes.").size());
      if (box_name.empty()) {
        throw std::invalid_argument("boxes child parameter name must not be empty");
      }

      named_values.emplace_back(box_name, load_box_values(parameter));
    }

    std::vector<Box> boxes;
    boxes.reserve(named_values.size());

    // 名前順に固定しておくと、設定ファイル編集後も起動ログと実処理順の追跡がしやすい。
    std::sort(
      named_values.begin(), named_values.end(),
      [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});

    // 各子パラメータの6値配列を既存のBox表現へ落とし込み、判定ロジックはそのまま再利用する。
    for (const auto & [box_name, values] : named_values) {
      if (values.size() != values_per_box) {
        throw std::invalid_argument(
                "boxes." + box_name +
                " must have 6 values: [x_min, x_max, y_min, y_max, z_min, z_max]");
      }
      boxes.push_back(
        Box{
          load_range(values[0], values[1], "x"),
          load_range(values[2], values[3], "y"),
          load_range(values[4], values[5], "z"),
        });
    }
    return boxes;
  }

  std::vector<double> load_box_values(const rclcpp::Parameter & parameter) const
  {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return parameter.as_double_array();
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      std::vector<double> values;
      const auto integer_values = parameter.as_integer_array();
      values.reserve(integer_values.size());

      // YAMLで整数だけを書いた場合も受け入れ、設定表現の自由度を保つ。
      for (const auto integer_value : integer_values) {
        values.push_back(static_cast<double>(integer_value));
      }
      return values;
    }

    throw std::invalid_argument(
            parameter.get_name() + " must be an array of 6 numeric values");
  }

  Range load_range(const double min, const double max, const std::string & axis) const
  {
    if (!std::isfinite(min) || !std::isfinite(max)) {
      throw std::invalid_argument("boxes must contain finite values");
    }
    if (min > max) {
      throw std::invalid_argument("boxes " + axis + " range must be ordered as [min, max]");
    }
    return Range{min, max};
  }

  void create_channels()
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
    for (std::size_t index = 0; index < input_topics_.size(); ++index) {
      const auto & input_topic = input_topics_[index];
      const auto & output_topic = output_topics_[index];

      Channel channel;
      channel.output_topic = output_topic;
      channel.publisher = create_publisher<CustomMsg>(output_topic, qos);
      channel.subscription = create_subscription<CustomMsg>(
        input_topic, qos,
        [this, input_topic](const CustomMsg::SharedPtr message) {
          this->handle_custom(input_topic, message);
        });

      channels_.emplace(input_topic, std::move(channel));
      RCLCPP_INFO(
        get_logger(), "Livox self filter channel: %s -> %s", input_topic.c_str(),
        output_topic.c_str());
    }
  }

  void handle_custom(const std::string & input_topic, const CustomMsg::SharedPtr message)
  {
    const auto channel_iter = channels_.find(input_topic);
    if (channel_iter == channels_.end() || message == nullptr) {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skip CustomMsg on %s because frame_id is empty.",
        input_topic.c_str());
      return;
    }

    // box未設定時はトピック差し替えだけを行い、launch/configを同じ構成で保てるようにする。
    if (boxes_.empty()) {
      channel_iter->second.publisher->publish(*message);
      return;
    }

    auto & channel = channel_iter->second;
    if (!ensure_filter_transform(channel, message->header.frame_id, input_topic)) {
      if (!drop_on_tf_failure_) {
        channel.publisher->publish(*message);
      }
      return;
    }

    auto output = filter_message(*message, channel.filter_from_lidar.value());
    channel.publisher->publish(std::move(output));
  }

  bool ensure_filter_transform(
    Channel & channel, const std::string & lidar_frame, const std::string & input_topic)
  {
    if (channel.filter_from_lidar.has_value() && channel.lidar_frame == lidar_frame) {
      return true;
    }

    try {
      // LiDARとbaseの静的関係を一度だけ取得し、以後は固定行列でfusion前の各点をbase判定する。
      const auto transform_message = tf_buffer_.lookupTransform(
        filter_frame_, lidar_frame, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
      channel.lidar_frame = lidar_frame;
      channel.filter_from_lidar = transform_from_message(transform_message.transform);
      RCLCPP_INFO(
        get_logger(), "Cached self filter TF %s -> %s for %s",
        filter_frame_.c_str(), lidar_frame.c_str(), input_topic.c_str());
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TF %s -> %s for %s: %s",
        filter_frame_.c_str(), lidar_frame.c_str(), input_topic.c_str(), error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid TF %s -> %s for %s: %s",
        filter_frame_.c_str(), lidar_frame.c_str(), input_topic.c_str(), error.what());
    }
    return false;
  }

  CustomMsg filter_message(
    const CustomMsg & message,
    const tf2::Transform & filter_from_lidar) const
  {
    CustomMsg output = message;
    output.points.clear();
    output.points.reserve(message.points.size());

    // 出力座標は元のLiDAR frameのまま保持し、base_footprint座標は除去判定だけに使う。
    for (const auto & point : message.points) {
      const tf2::Vector3 lidar_point(point.x, point.y, point.z);
      const tf2::Vector3 filter_point = filter_from_lidar * lidar_point;
      if (!is_inside_any_box(filter_point)) {
        output.points.push_back(point);
      }
    }

    output.point_num = static_cast<std::uint32_t>(output.points.size());
    return output;
  }

  bool is_inside_any_box(const tf2::Vector3 & point) const
  {
    for (const auto & box : boxes_) {
      if (contains(box.x, point.x()) && contains(box.y, point.y()) && contains(box.z, point.z())) {
        return true;
      }
    }
    return false;
  }

  bool contains(const Range & range, const double value) const
  {
    return (range.min - margin_) <= value && value <= (range.max + margin_);
  }

  std::vector<std::string> input_topics_;
  std::vector<std::string> output_topics_;
  std::string filter_frame_;
  double margin_{0.03};
  double tf_timeout_sec_{0.1};
  bool drop_on_tf_failure_{true};
  std::vector<Box> boxes_;
  std::unordered_map<std::string, Channel> channels_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    // multi_lidar_fusion.yamlのboxesネストを通常パラメータとして列挙できるようにする。
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<ai_ship_robot_slam::LivoxCustomMsgSelfFilterNode>(options));
  } catch (const std::exception & exc) {
    fprintf(stderr, "livox_custommsg_self_filter_node failed: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
