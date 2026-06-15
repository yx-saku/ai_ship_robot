#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace ai_ship_robot_slam
{
namespace
{
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

template<typename T>
T read_unaligned(const std::vector<std::uint8_t> & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

void write_float(std::vector<std::uint8_t> & data, const std::size_t offset, const float value)
{
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

void write_uint16(std::vector<std::uint8_t> & data, const std::size_t offset, const std::uint16_t value)
{
  std::memcpy(data.data() + offset, &value, sizeof(std::uint16_t));
}

std::uint32_t datatype_size(const std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

const PointField * find_field(
  const PointCloud2 & message, const std::initializer_list<const char *> names)
{
  for (const auto & name : names) {
    const auto field = std::find_if(
      message.fields.begin(), message.fields.end(),
      [name](const PointField & candidate) { return candidate.name == name; });
    if (field != message.fields.end()) {
      return &(*field);
    }
  }
  return nullptr;
}

double read_field_as_double(
  const PointCloud2 & message, const PointField & field, const std::size_t point_offset)
{
  const auto offset = point_offset + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      return read_unaligned<std::int8_t>(message.data, offset);
    case PointField::UINT8:
      return read_unaligned<std::uint8_t>(message.data, offset);
    case PointField::INT16:
      return read_unaligned<std::int16_t>(message.data, offset);
    case PointField::UINT16:
      return read_unaligned<std::uint16_t>(message.data, offset);
    case PointField::INT32:
      return read_unaligned<std::int32_t>(message.data, offset);
    case PointField::UINT32:
      return read_unaligned<std::uint32_t>(message.data, offset);
    case PointField::FLOAT32:
      return read_unaligned<float>(message.data, offset);
    case PointField::FLOAT64:
      return read_unaligned<double>(message.data, offset);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
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

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}
}  // namespace

class Mid360LioSamPointCloudAdapterNode : public rclcpp::Node
{
public:
  Mid360LioSamPointCloudAdapterNode()
  : Node("mid360_lio_sam_pointcloud_adapter_node")
  {
    // 入出力topicと補完値をパラメータ化し、実機driverとGazebo pluginの差分をlaunch側で吸収する。
    input_points_topic_ = declare_parameter<std::string>("input_points_topic", "/left_lidar/points");
    output_points_topic_ = declare_parameter<std::string>("output_points_topic", "/left_lidar/lio_sam_points");
    output_frame_ = declare_parameter<std::string>("output_frame", "");
    derived_ring_count_ = declare_parameter<int>("derived_ring_count", 4);
    min_vertical_angle_deg_ = declare_parameter<double>("min_vertical_angle_deg", -7.22);
    max_vertical_angle_deg_ = declare_parameter<double>("max_vertical_angle_deg", 55.22);
    timestamp_unit_scale_ = declare_parameter<double>("timestamp_unit_scale", 1.0e-9);
    default_point_time_sec_ = declare_parameter<double>("default_point_time_sec", 0.0);
    max_relative_time_sec_ = declare_parameter<double>("max_relative_time_sec", 0.2);

    // LiDAR点群は欠落より遅延回避を優先し、Livox driverと同じSensorDataQoSで中継する。
    const auto qos = rclcpp::SensorDataQoS();
    points_pub_ = create_publisher<PointCloud2>(output_points_topic_, qos);
    points_sub_ = create_subscription<PointCloud2>(
      input_points_topic_, qos,
      std::bind(&Mid360LioSamPointCloudAdapterNode::handle_points, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Adapt Mid-360 point cloud for LIO-SAM: input=%s output=%s",
      input_points_topic_.c_str(), output_points_topic_.c_str());
  }

private:
  void handle_points(const PointCloud2::SharedPtr input)
  {
    if (!validate_layout(*input)) {
      return;
    }

    const auto * x_field = find_field(*input, {"x"});
    const auto * y_field = find_field(*input, {"y"});
    const auto * z_field = find_field(*input, {"z"});
    const auto * intensity_field = find_field(*input, {"intensity", "reflectivity"});
    const auto * ring_field = find_field(*input, {"ring", "line", "laser_id", "channel"});
    const auto * time_field = find_field(*input, {"offset_time", "time", "timestamp", "t"});

    // LIO-SAMのPointXYZIRTが期待するフィールド名とoffsetへ詰め替え、PCL側の型解決を安定させる。
    PointCloud2 output;
    output.header = input->header;
    if (!output_frame_.empty()) {
      output.header.frame_id = output_frame_;
    }
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = kOutputPointStep;
    output.fields = {
      make_field("x", kXOffset, PointField::FLOAT32),
      make_field("y", kYOffset, PointField::FLOAT32),
      make_field("z", kZOffset, PointField::FLOAT32),
      make_field("intensity", kIntensityOffset, PointField::FLOAT32),
      make_field("ring", kRingOffset, PointField::UINT16),
      make_field("time", kTimeOffset, PointField::FLOAT32),
    };
    output.data.resize(static_cast<std::size_t>(input->width) * input->height * kOutputPointStep);

    std::uint32_t output_count = 0;
    for (std::uint32_t row = 0; row < input->height; ++row) {
      for (std::uint32_t column = 0; column < input->width; ++column) {
        const auto input_offset = static_cast<std::size_t>(row) * input->row_step +
          static_cast<std::size_t>(column) * input->point_step;
        output_count += append_point(
          *input, input_offset, *x_field, *y_field, *z_field, intensity_field, ring_field, time_field,
          output, output_count);
      }
    }

    // 無効点を除外した後の実点数でメタデータを更新し、LIO-SAMのdense前提に合わせる。
    output.width = output_count;
    output.row_step = output.point_step * output.width;
    output.data.resize(static_cast<std::size_t>(output.row_step));
    if (output.width == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skip empty adapted point cloud.");
      return;
    }

    points_pub_->publish(output);
  }

  bool validate_layout(const PointCloud2 & message)
  {
    if (message.is_bigendian) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported.");
      return false;
    }

    const auto * x_field = find_field(message, {"x"});
    const auto * y_field = find_field(message, {"y"});
    const auto * z_field = find_field(message, {"z"});
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 must contain x, y and z fields.");
      return false;
    }

    // 各fieldがpoint_step内へ収まることを確認し、破損したPointCloud2で範囲外参照しない。
    for (const auto & field : message.fields) {
      const auto size = datatype_size(field.datatype);
      if (size == 0 || field.count == 0 || field.offset + size * field.count > message.point_step) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Unsupported or invalid PointCloud2 field: %s",
          field.name.c_str());
        return false;
      }
    }

    const auto required_size = static_cast<std::size_t>(message.row_step) * message.height;
    if (message.data.size() < required_size) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 data buffer is shorter than layout.");
      return false;
    }
    return true;
  }

  std::uint32_t append_point(
    const PointCloud2 & input, const std::size_t input_offset, const PointField & x_field,
    const PointField & y_field, const PointField & z_field, const PointField * intensity_field,
    const PointField * ring_field, const PointField * time_field, PointCloud2 & output,
    const std::uint32_t output_index) const
  {
    const auto x = read_field_as_double(input, x_field, input_offset);
    const auto y = read_field_as_double(input, y_field, input_offset);
    const auto z = read_field_as_double(input, z_field, input_offset);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return 0;
    }

    // 欠落しやすいintensity/ring/timeは、既存フィールド優先で読み、なければLIO-SAMが落ちない値を補う。
    const auto intensity = intensity_field != nullptr ?
      static_cast<float>(read_field_as_double(input, *intensity_field, input_offset)) : 0.0F;
    const auto ring = ring_field != nullptr ?
      read_ring(input, *ring_field, input_offset) : derive_ring(x, y, z);
    const auto relative_time = time_field != nullptr ?
      read_relative_time(input, *time_field, input_offset) : default_point_time_sec_;

    const auto output_offset = static_cast<std::size_t>(output_index) * kOutputPointStep;
    write_float(output.data, output_offset + kXOffset, static_cast<float>(x));
    write_float(output.data, output_offset + kYOffset, static_cast<float>(y));
    write_float(output.data, output_offset + kZOffset, static_cast<float>(z));
    write_float(output.data, output_offset + kIntensityOffset, intensity);
    write_uint16(output.data, output_offset + kRingOffset, ring);
    write_float(output.data, output_offset + kTimeOffset, static_cast<float>(relative_time));
    return 1;
  }

  std::uint16_t read_ring(
    const PointCloud2 & input, const PointField & ring_field, const std::size_t input_offset) const
  {
    const auto raw_ring = read_field_as_double(input, ring_field, input_offset);
    if (!std::isfinite(raw_ring) || raw_ring < 0.0) {
      return 0;
    }
    const auto clamped = std::clamp(raw_ring, 0.0, static_cast<double>(std::numeric_limits<std::uint16_t>::max()));
    return static_cast<std::uint16_t>(std::lround(clamped));
  }

  std::uint16_t derive_ring(const double x, const double y, const double z) const
  {
    if (derived_ring_count_ <= 1 || max_vertical_angle_deg_ <= min_vertical_angle_deg_) {
      return 0;
    }

    // ringフィールドがないsimulation点群では、垂直角から疑似ringを作り投影処理を継続させる。
    const auto horizontal_range = std::hypot(x, y);
    if (horizontal_range <= std::numeric_limits<double>::epsilon()) {
      return 0;
    }
    const auto vertical_angle = std::atan2(z, horizontal_range) * 180.0 / kPi;
    const auto ratio = (vertical_angle - min_vertical_angle_deg_) /
      (max_vertical_angle_deg_ - min_vertical_angle_deg_);
    const auto ring = std::lround(std::clamp(ratio, 0.0, 1.0) * static_cast<double>(derived_ring_count_ - 1));
    return static_cast<std::uint16_t>(ring);
  }

  double read_relative_time(
    const PointCloud2 & input, const PointField & time_field, const std::size_t input_offset) const
  {
    const auto raw_time = read_field_as_double(input, time_field, input_offset);
    if (!std::isfinite(raw_time) || raw_time < 0.0) {
      return default_point_time_sec_;
    }

    // Livox系の整数時刻はns単位のことが多く、絶対時刻ならheader stampとの差分へ変換する。
    auto relative_time = raw_time;
    if (time_field.datatype != PointField::FLOAT32 && time_field.datatype != PointField::FLOAT64) {
      relative_time = raw_time * timestamp_unit_scale_;
    }
    if (relative_time > 1000.0) {
      relative_time -= stamp_to_seconds(input.header.stamp);
    }
    if (relative_time < 0.0 || relative_time > max_relative_time_sec_) {
      return default_point_time_sec_;
    }
    return relative_time;
  }

  std::string input_points_topic_;
  std::string output_points_topic_;
  std::string output_frame_;
  int derived_ring_count_;
  double min_vertical_angle_deg_;
  double max_vertical_angle_deg_;
  double timestamp_unit_scale_;
  double default_point_time_sec_;
  double max_relative_time_sec_;
  rclcpp::Subscription<PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr points_pub_;
};
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ai_ship_robot_slam::Mid360LioSamPointCloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
