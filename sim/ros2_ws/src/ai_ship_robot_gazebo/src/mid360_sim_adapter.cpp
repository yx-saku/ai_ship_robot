#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

constexpr int kAngleScale = 1000;
constexpr int kFullCircleKey = 360 * kAngleScale;
constexpr double kPi = 3.14159265358979323846;

struct DirectionKey
{
  int azimuth{};
  int zenith{};

  bool operator==(const DirectionKey & other) const
  {
    return azimuth == other.azimuth && zenith == other.zenith;
  }
};

struct DirectionKeyHash
{
  std::size_t operator()(const DirectionKey & key) const
  {
    const auto mixed = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.azimuth)) << 32) ^
      static_cast<std::uint32_t>(key.zenith);
    return std::hash<std::uint64_t>{}(mixed);
  }
};

std::string default_scan_pattern_csv_file()
{
  return ament_index_cpp::get_package_share_directory("ros2_livox_simulation") + "/scan_mode/mid360.csv";
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
  auto normalized = azimuth_mdeg % kFullCircleKey;
  if (normalized < 0) {
    normalized += kFullCircleKey;
  }
  return normalized;
}

DirectionKey make_scan_pattern_key(const double azimuth_deg, const double zenith_deg)
{
  return DirectionKey{
    normalize_azimuth_key(static_cast<int>(std::llround(normalize_degrees(azimuth_deg) * kAngleScale))),
    static_cast<int>(std::llround(zenith_deg * kAngleScale))};
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> cells;
  std::string cell;
  std::stringstream stream(line);
  while (std::getline(stream, cell, ',')) {
    cells.push_back(cell);
  }
  return cells;
}

int find_column(const std::vector<std::string> & header, const std::initializer_list<const char *> names)
{
  for (std::size_t index = 0; index < header.size(); ++index) {
    for (const auto * name : names) {
      if (header[index] == name) {
        return static_cast<int>(index);
      }
    }
  }
  return -1;
}

}  // namespace

namespace ai_ship_robot_gazebo
{

class Mid360SimAdapter : public rclcpp::Node
{
public:
  Mid360SimAdapter()
  : Node("mid360_sim_adapter")
  {
    input_custom_topic_ = declare_parameter<std::string>("input_custom_topic", "/left_lidar/custom");
    input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/left_lidar/imu");
    output_custom_topic_ = declare_parameter<std::string>("output_custom_topic", "/livox/lidar");
    output_imu_topic_ = declare_parameter<std::string>("output_imu_topic", "/livox/imu");
    output_lidar_frame_ = declare_parameter<std::string>("output_lidar_frame", "left_lidar_link");
    output_imu_frame_ = declare_parameter<std::string>("output_imu_frame", "left_lidar_imu_link");
    gravity_ = declare_parameter<double>("gravity", 9.80511);
    convert_imu_acceleration_to_g_ = declare_parameter<bool>("convert_imu_acceleration_to_g", true);
    synthesize_line_from_pattern_ = declare_parameter<bool>("synthesize_line_from_pattern", true);
    use_scan_pattern_line_lookup_ = declare_parameter<bool>("use_scan_pattern_line_lookup", false);
    force_zero_offset_time_ = declare_parameter<bool>("force_zero_offset_time", false);
    scan_pattern_csv_file_ = declare_parameter<std::string>("scan_pattern_csv_file", default_scan_pattern_csv_file());
    scan_pattern_physical_line_count_ = declare_parameter<int>("scan_pattern_physical_line_count", 4);
    synthetic_line_count_ = declare_parameter<int>("synthetic_line_count", 4);
    lidar_qos_depth_ = declare_parameter<int>("lidar_qos_depth", 200);
    imu_qos_depth_ = declare_parameter<int>("imu_qos_depth", 1000);

    if (gravity_ <= 0.0) {
      throw std::invalid_argument("gravity must be positive.");
    }
    if (lidar_qos_depth_ <= 0 || imu_qos_depth_ <= 0) {
      throw std::invalid_argument("qos depth parameters must be positive.");
    }
    if (use_scan_pattern_line_lookup_) {
      load_scan_pattern();
    } else if (synthesize_line_from_pattern_) {
      RCLCPP_INFO(get_logger(), "Using index-based synthetic Livox line assignment.");
    }

    // Gazebo sensor入力はbest-effortで受け、rosbag対象のLivox互換出力だけreliableにする。
    const auto input_lidar_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(lidar_qos_depth_)))
      .best_effort()
      .durability_volatile();
    const auto input_imu_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(imu_qos_depth_)))
      .best_effort()
      .durability_volatile();
    const auto output_lidar_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(lidar_qos_depth_)))
      .reliable()
      .durability_volatile();
    const auto output_imu_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(imu_qos_depth_)))
      .reliable()
      .durability_volatile();

    custom_publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_custom_topic_, output_lidar_qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, output_imu_qos);
    custom_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_custom_topic_, input_lidar_qos,
      std::bind(&Mid360SimAdapter::custom_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, input_imu_qos, std::bind(&Mid360SimAdapter::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Mid-360 sim adapter: %s -> %s, %s -> %s", input_custom_topic_.c_str(),
      output_custom_topic_.c_str(), input_imu_topic_.c_str(), output_imu_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Mid-360 sim adapter QoS: input best_effort, output reliable, lidar_depth=%d, imu_depth=%d", lidar_qos_depth_,
      imu_qos_depth_);
  }

private:
  void load_scan_pattern()
  {
    if (!synthesize_line_from_pattern_ || !use_scan_pattern_line_lookup_) {
      return;
    }

    std::ifstream file(scan_pattern_csv_file_);
    if (!file) {
      throw std::runtime_error("scan pattern csv not found: " + scan_pattern_csv_file_);
    }

    std::string line;
    if (!std::getline(file, line)) {
      throw std::runtime_error("scan pattern csv has no header: " + scan_pattern_csv_file_);
    }

    const auto header = split_csv_line(line);
    const int azimuth_column = find_column(header, {"Azimuth/deg", "Azimuth"});
    const int zenith_column = find_column(header, {"Zenith/deg", "Zenith"});
    if (azimuth_column < 0 || zenith_column < 0) {
      throw std::runtime_error("scan pattern csv is missing Azimuth/Zenith columns: " + scan_pattern_csv_file_);
    }

    // CSV行順はLivox照射順なので、物理line内の進行順から疑似lineを均等に割り当てる。
    int collision_count = 0;
    const auto physical_line_count = std::max(1, scan_pattern_physical_line_count_);
    const auto synthetic_line_count = std::max(1, synthetic_line_count_);
    std::size_t pattern_index = 0;
    scan_pattern_line_by_direction_.reserve(900000);
    while (std::getline(file, line)) {
      const auto cells = split_csv_line(line);
      if (static_cast<int>(cells.size()) <= std::max(azimuth_column, zenith_column)) {
        continue;
      }
      try {
        const auto key = make_scan_pattern_key(std::stod(cells[azimuth_column]), std::stod(cells[zenith_column]));
        const auto synthetic_line = static_cast<std::uint8_t>((pattern_index / physical_line_count) % synthetic_line_count);
        const auto [iter, inserted] = scan_pattern_line_by_direction_.emplace(key, synthetic_line);
        if (!inserted && iter->second != synthetic_line) {
          ++collision_count;
        }
        ++pattern_index;
      } catch (const std::exception &) {
        continue;
      }
    }

    if (scan_pattern_line_by_direction_.empty()) {
      throw std::runtime_error("scan pattern csv has no usable directions: " + scan_pattern_csv_file_);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu Livox scan pattern directions.", scan_pattern_line_by_direction_.size());
    if (collision_count > 0) {
      RCLCPP_WARN(get_logger(), "Livox scan pattern had %d direction key collisions.", collision_count);
    }
  }

  void custom_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    livox_ros_driver2::msg::CustomMsg output;
    output.header.stamp = message->header.stamp;
    output.header.frame_id = output_lidar_frame_.empty() ? message->header.frame_id : output_lidar_frame_;
    output.timebase = message->timebase;
    output.lidar_id = message->lidar_id;
    output.rsvd = message->rsvd;

    const auto input_size = message->points.size();
    const auto point_count = message->point_num > 0 ? std::min<std::size_t>(input_size, message->point_num) : input_size;
    output.points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      output.points.push_back(convert_point(message->points[index], index));
    }
    output.point_num = static_cast<std::uint32_t>(output.points.size());
    custom_publisher_->publish(std::move(output));
  }

  livox_ros_driver2::msg::CustomPoint convert_point(
    const livox_ros_driver2::msg::CustomPoint & point, const std::size_t point_index) const
  {
    livox_ros_driver2::msg::CustomPoint output;
    output.offset_time = force_zero_offset_time_ ? 0U : point.offset_time;
    output.x = point.x;
    output.y = point.y;
    output.z = point.z;
    output.reflectivity = point.reflectivity;
    output.tag = point.tag;
    output.line = output_line(point, point_index);
    return output;
  }

  std::uint8_t output_line(const livox_ros_driver2::msg::CustomPoint & point, const std::size_t point_index) const
  {
    if (synthesize_line_from_pattern_ && point.line == 0 && synthetic_line_count_ > 1) {
      if (use_scan_pattern_line_lookup_) {
        // 精密だが重い方向逆引きは必要時だけ残し、通常は高速なindex割り当てを使う。
        const auto line = lookup_scan_pattern_line(point);
        return line.value_or(fallback_line(point_index));
      }
      return fallback_line(point_index);
    }
    return point.line;
  }

  std::uint8_t fallback_line(const std::size_t point_index) const
  {
    // scan pattern CSVの行順と同じ疑似line規則にし、range image上の点配置を安定させる。
    const auto physical_line_count = static_cast<std::size_t>(std::max(1, scan_pattern_physical_line_count_));
    const auto synthetic_line_count = static_cast<std::size_t>(std::max(1, synthetic_line_count_));
    return static_cast<std::uint8_t>((point_index / physical_line_count) % synthetic_line_count);
  }

  std::optional<std::uint8_t> lookup_scan_pattern_line(const livox_ros_driver2::msg::CustomPoint & point) const
  {
    const auto horizontal_range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    const auto point_range = std::hypot(horizontal_range, static_cast<double>(point.z));
    if (point_range <= 1.0e-12) {
      return std::nullopt;
    }

    // pluginのray方向をCSVのAzimuth/Zenithへ戻し、float丸めに備えて近傍ミリ度だけを探索する。
    const auto azimuth_deg = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) * 180.0 / kPi;
    const auto zenith_deg = 90.0 - std::atan2(static_cast<double>(point.z), horizontal_range) * 180.0 / kPi;
    const auto key = make_scan_pattern_key(azimuth_deg, zenith_deg);
    for (int zenith_delta = -1; zenith_delta <= 1; ++zenith_delta) {
      for (int azimuth_delta = -1; azimuth_delta <= 1; ++azimuth_delta) {
        const DirectionKey candidate{normalize_azimuth_key(key.azimuth + azimuth_delta), key.zenith + zenith_delta};
        const auto iter = scan_pattern_line_by_direction_.find(candidate);
        if (iter != scan_pattern_line_by_direction_.end()) {
          return iter->second;
        }
      }
    }
    return std::nullopt;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    sensor_msgs::msg::Imu output = *message;
    output.header.frame_id = output_imu_frame_.empty() ? message->header.frame_id : output_imu_frame_;

    // ROS標準のm/s^2をG単位へ変換し、UV-Lab版LIO-SAMのimuConverter前提に合わせる。
    const double acceleration_scale = convert_imu_acceleration_to_g_ ? 1.0 / gravity_ : 1.0;
    output.linear_acceleration.x = message->linear_acceleration.x * acceleration_scale;
    output.linear_acceleration.y = message->linear_acceleration.y * acceleration_scale;
    output.linear_acceleration.z = message->linear_acceleration.z * acceleration_scale;
    if (output.linear_acceleration_covariance[0] != -1.0) {
      const double covariance_scale = acceleration_scale * acceleration_scale;
      for (auto & value : output.linear_acceleration_covariance) {
        value *= covariance_scale;
      }
    }
    imu_publisher_->publish(std::move(output));
  }

  std::string input_custom_topic_;
  std::string input_imu_topic_;
  std::string output_custom_topic_;
  std::string output_imu_topic_;
  std::string output_lidar_frame_;
  std::string output_imu_frame_;
  double gravity_{};
  bool convert_imu_acceleration_to_g_{};
  bool synthesize_line_from_pattern_{};
  bool use_scan_pattern_line_lookup_{};
  bool force_zero_offset_time_{};
  std::string scan_pattern_csv_file_;
  int scan_pattern_physical_line_count_{};
  int synthetic_line_count_{};
  int lidar_qos_depth_{};
  int imu_qos_depth_{};
  std::unordered_map<DirectionKey, std::uint8_t, DirectionKeyHash> scan_pattern_line_by_direction_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace ai_ship_robot_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ai_ship_robot_gazebo::Mid360SimAdapter>());
  } catch (const std::exception & exc) {
    fprintf(stderr, "mid360_sim_adapter failed: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
