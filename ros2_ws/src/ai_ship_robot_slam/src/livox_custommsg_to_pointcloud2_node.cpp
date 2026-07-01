#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ai_ship_robot_slam
{

class LivoxCustomMsgToPointCloud2Node : public rclcpp::Node
{
public:
  LivoxCustomMsgToPointCloud2Node()
  : Node("livox_custommsg_to_pointcloud2_node")
  {
    discovery_period_sec_ = declare_parameter<double>("discovery_period_sec", 1.0);
    qos_reliable_ = declare_parameter<bool>("qos_reliable", false);
    qos_depth_ = declare_parameter<int>("qos_depth", 10);

    if (qos_depth_ <= 0) {
      throw std::invalid_argument("qos_depth must be greater than 0.");
    }

    RCLCPP_INFO(
      get_logger(), "Dynamic CustomMsg->PointCloud2 bridge QoS: mode=%s depth=%d",
      qos_reliable_ ? "reliable" : "best_effort", qos_depth_);

    // 可視化要求があるtopicだけをbridgeするため、ROS graphを周期探索して動的に接続を更新する。
    discovery_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(discovery_period_sec_, 0.1))),
      [this]() { this->update_bridges(); });
  }

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointField = sensor_msgs::msg::PointField;

  struct Channel
  {
    std::string input_topic;
    std::string output_topic;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher;
    rclcpp::Subscription<CustomMsg>::SharedPtr subscription;
  };

  static constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
  static constexpr const char * kCustomMsgType = "livox_ros_driver2/msg/CustomMsg";
  static constexpr const char * kPointsSuffix = "/points";

  static bool ends_with(const std::string & value, const std::string & suffix)
  {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  static bool contains_type(
    const std::vector<std::string> & types, const std::string & expected_type)
  {
    return std::find(types.begin(), types.end(), expected_type) != types.end();
  }

  static std::string base_topic_from_points_topic(const std::string & output_topic)
  {
    return output_topic.substr(0, output_topic.size() - std::strlen(kPointsSuffix));
  }

  static PointField make_field(const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
  {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    return field;
  }

  static void write_float32(std::vector<std::uint8_t> & data, const std::size_t offset, const float value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(value));
  }

  static void write_uint16(std::vector<std::uint8_t> & data, const std::size_t offset, const std::uint16_t value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(value));
  }

  static void write_uint32(std::vector<std::uint8_t> & data, const std::size_t offset, const std::uint32_t value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(value));
  }

  std::vector<PointField> point_fields() const
  {
    return {
      make_field("x", 0, PointField::FLOAT32),
      make_field("y", 4, PointField::FLOAT32),
      make_field("z", 8, PointField::FLOAT32),
      make_field("intensity", 12, PointField::FLOAT32),
      make_field("ring", 16, PointField::UINT16),
      make_field("time", 18, PointField::UINT32),
    };
  }

  rclcpp::QoS bridge_qos() const
  {
    // 上流publisherとの接続性と可視化出力のQoSを同一パラメータで制御する。
    auto qos = rclcpp::QoS(
      rclcpp::KeepLast(static_cast<std::size_t>(qos_depth_))).durability_volatile();
    return qos_reliable_ ? qos.reliable() : qos.best_effort();
  }

  void create_bridge(const std::string & input_topic, const std::string & output_topic)
  {
    const auto qos = bridge_qos();

    Channel channel;
    channel.input_topic = input_topic;
    channel.output_topic = output_topic;
    channel.publisher = create_publisher<PointCloud2>(output_topic, qos);
    channel.subscription = create_subscription<CustomMsg>(
      input_topic, qos,
      [this, output_topic](const CustomMsg::SharedPtr message) {
        this->handle_custom(output_topic, message);
      });
    channels_.emplace(output_topic, std::move(channel));
    RCLCPP_INFO(
      get_logger(),
      "Activated dynamic CustomMsg->PointCloud2 bridge: %s -> %s (mode=%s depth=%d)",
      input_topic.c_str(), output_topic.c_str(), qos_reliable_ ? "reliable" : "best_effort",
      qos_depth_);
  }

  void remove_bridge(const std::string & output_topic)
  {
    const auto channel_iter = channels_.find(output_topic);
    if (channel_iter == channels_.end()) {
      return;
    }

    // 条件を満たさなくなったbridgeはpublisher/subscriptionを明示破棄し、無駄なgraph接続を残さない。
    channel_iter->second.subscription.reset();
    channel_iter->second.publisher.reset();
    RCLCPP_INFO(
      get_logger(), "Deactivated dynamic CustomMsg->PointCloud2 bridge: %s -> %s",
      channel_iter->second.input_topic.c_str(), output_topic.c_str());
    channels_.erase(channel_iter);
  }

  void update_bridges()
  {
    const auto topic_names_and_types = get_topic_names_and_types();
    std::unordered_set<std::string> desired_output_topics;

    // /points購読要求と対応するCustomMsg publisherの両方があるtopicだけをbridge対象として採用する。
    for (const auto & [topic_name, topic_types] : topic_names_and_types) {
      if (!ends_with(topic_name, kPointsSuffix)) {
        continue;
      }
      if (!contains_type(topic_types, kPointCloud2Type)) {
        continue;
      }
      if (count_subscribers(topic_name) == 0U) {
        continue;
      }

      const auto input_topic = base_topic_from_points_topic(topic_name);
      const auto input_iter = topic_names_and_types.find(input_topic);
      if (input_iter == topic_names_and_types.end()) {
        continue;
      }
      if (!contains_type(input_iter->second, kCustomMsgType)) {
        continue;
      }
      if (count_publishers(input_topic) == 0U) {
        continue;
      }

      desired_output_topics.insert(topic_name);
      if (channels_.find(topic_name) == channels_.end()) {
        create_bridge(input_topic, topic_name);
      }
    }

    std::vector<std::string> stale_output_topics;
    stale_output_topics.reserve(channels_.size());
    for (const auto & [output_topic, channel] : channels_) {
      (void)channel;
      if (desired_output_topics.find(output_topic) == desired_output_topics.end()) {
        stale_output_topics.push_back(output_topic);
      }
    }
    for (const auto & output_topic : stale_output_topics) {
      remove_bridge(output_topic);
    }
  }

  void handle_custom(const std::string & output_topic, const CustomMsg::SharedPtr message)
  {
    const auto channel_iter = channels_.find(output_topic);
    if (channel_iter == channels_.end()) {
      return;
    }
    if (channel_iter->second.publisher == nullptr) {
      return;
    }

    // graph更新とcallback実行のずれで購読者が消えた直後でも、不要なPointCloud2展開を避ける。
    if (channel_iter->second.publisher->get_subscription_count() == 0U) {
      return;
    }

    PointCloud2 output;
    output.header = message->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(message->points.size());
    output.fields = point_fields();
    output.is_bigendian = false;
    output.is_dense = false;
    output.point_step = 22;
    output.row_step = output.point_step * output.width;
    output.data.resize(output.row_step);

    // 可視化用ノードではCustomMsgをそのままPointCloud2へ展開し、元topicの末尾へ/pointsを付けて出す。
    for (std::size_t index = 0; index < message->points.size(); ++index) {
      const auto & point = message->points[index];
      const auto base = index * output.point_step;
      write_float32(output.data, base + 0, point.x);
      write_float32(output.data, base + 4, point.y);
      write_float32(output.data, base + 8, point.z);
      write_float32(output.data, base + 12, static_cast<float>(point.reflectivity));
      write_uint16(output.data, base + 16, point.line);
      write_uint32(output.data, base + 18, point.offset_time);
    }

    channel_iter->second.publisher->publish(std::move(output));
  }

  double discovery_period_sec_{};
  bool qos_reliable_{false};
  int qos_depth_{10};
  std::unordered_map<std::string, Channel> channels_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
};

}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ai_ship_robot_slam::LivoxCustomMsgToPointCloud2Node>());
  } catch (const std::exception & exc) {
    fprintf(stderr, "livox_custommsg_to_pointcloud2_node failed: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
