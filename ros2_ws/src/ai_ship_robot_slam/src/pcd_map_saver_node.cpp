#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace ai_ship_robot_slam
{
namespace
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Trigger = std_srvs::srv::Trigger;

struct MapPoint
{
  float x{};
  float y{};
  float z{};
  float intensity{};
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
      "cloud_topic", "/lio_sam/mapping/cloud_registered");

    // LIO-SAMの登録済み点群だけを購読し、保存用の内部バッファへ蓄積する。
    cloud_subscription_ = this->create_subscription<PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const PointCloud2::SharedPtr message) { this->handle_cloud(message); });

    // 保存操作は固定サービス名に限定し、外部から必要なタイミングでPCDを書き出す。
    save_service_ = this->create_service<Trigger>(
      "save_pcd_map",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        this->save_map(response);
      });
  }

  ~PcdMapSaverNode() override
  {
    if (map_points_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No accumulated cloud points to save on shutdown.");
      return;
    }

    // 終了時にも現在までの蓄積結果を保存し、サービス呼び忘れによる地図消失を防ぐ。
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
    PointCloud2 transformed_cloud;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id, message->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*message, transformed_cloud, transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Failed to transform cloud to %s: %s", target_frame_.c_str(), error.what());
      return;
    }

    // map frameへ変換できた点群からPCD保存に必要な最小フィールドだけを取り出す。
    const auto * x_field = find_field(transformed_cloud, "x");
    const auto * y_field = find_field(transformed_cloud, "y");
    const auto * z_field = find_field(transformed_cloud, "z");
    const auto * intensity_field = find_field(transformed_cloud, "intensity");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Cloud does not contain x/y/z fields.");
      return;
    }

    // organized/unorganizedのどちらでも、PointCloud2のpoint_step単位で全点を順に蓄積する。
    const auto point_count = static_cast<std::size_t>(transformed_cloud.width) * transformed_cloud.height;
    map_points_.reserve(map_points_.size() + point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      const auto point_offset = index * transformed_cloud.point_step;
      MapPoint point;
      point.x = read_numeric_field(transformed_cloud, *x_field, point_offset);
      point.y = read_numeric_field(transformed_cloud, *y_field, point_offset);
      point.z = read_numeric_field(transformed_cloud, *z_field, point_offset);
      point.intensity = intensity_field == nullptr ? 0.0F : read_numeric_field(
        transformed_cloud, *intensity_field, point_offset);
      map_points_.push_back(point);
    }
  }

  void save_map(const Trigger::Response::SharedPtr response)
  {
    response->success = save_map_file(response->message);
  }

  bool save_map_file(std::string & message)
  {
    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {
      message = "failed to create output directory: " + error.message();
      return false;
    }

    // 蓄積済み点群を標準的なASCII PCDとして保存し、外部ツールで確認しやすくする。
    const auto output_path = std::filesystem::path(output_directory_) /
      ("lio_sam_map_" + timestamp_string() + ".pcd");
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
    file << "WIDTH " << map_points_.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << map_points_.size() << "\n";
    file << "DATA ascii\n";
    file << std::fixed << std::setprecision(6);
    for (const auto & point : map_points_) {
      file << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << '\n';
    }

    message = output_path.string();
    return true;
  }

  std::string target_frame_;
  std::string output_directory_;
  std::string cloud_topic_;
  std::vector<MapPoint> map_points_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscription_;
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
