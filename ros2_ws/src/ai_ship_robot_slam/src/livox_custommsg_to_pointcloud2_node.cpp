#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/lidar1/custom", "/lidar2/custom", "/lidar3/custom", "/lidar4/custom",
        "/lidar1/livox/lidar", "/lidar2/livox/lidar", "/lidar3/livox/lidar", "/lidar4/livox/lidar"});

    if (input_topics_.empty()) {
      throw std::invalid_argument("input_topics must contain at least one topic.");
    }

    const auto qos = rclcpp::SensorDataQoS();
    for (const auto & input_topic : input_topics_) {
      if (input_topic.empty() || input_topic.back() == '/') {
        throw std::invalid_argument("input_topics must not be empty or end with '/'.");
      }

      Channel channel;
      channel.output_topic = input_topic + "/points";
      channel.publisher = create_publisher<PointCloud2>(channel.output_topic, qos);
      channel.subscription = create_subscription<CustomMsg>(
        input_topic, qos,
        [this, input_topic](const CustomMsg::SharedPtr message) {
          this->handle_custom(input_topic, message);
        });
      channels_.emplace(input_topic, std::move(channel));
      RCLCPP_INFO(
        get_logger(), "CustomMsg->PointCloud2 bridge: %s -> %s", input_topic.c_str(),
        (input_topic + "/points").c_str());
    }
  }

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointField = sensor_msgs::msg::PointField;

  struct Channel
  {
    std::string output_topic;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher;
    rclcpp::Subscription<CustomMsg>::SharedPtr subscription;
  };

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

  void handle_custom(const std::string & input_topic, const CustomMsg::SharedPtr message)
  {
    const auto channel_iter = channels_.find(input_topic);
    if (channel_iter == channels_.end()) {
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

  std::vector<std::string> input_topics_;
  std::unordered_map<std::string, Channel> channels_;
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
