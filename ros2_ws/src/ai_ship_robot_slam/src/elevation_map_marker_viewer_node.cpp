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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ai_ship_robot_slam
{
namespace
{

constexpr const char * kPointCloudTopic = "/elevation_map/z_max_points";
constexpr const char * kCsvFileName = "global_elevation_map.csv";
constexpr const char * kManifestFileName = "elevation_manifest.yaml";
constexpr double kFallbackCellSize = 0.01;
constexpr double kPublishPeriodSec = 1.0;

struct ElevationCell
{
  double x{};
  double y{};
  double z_max{};
};

struct ManifestInfo
{
  std::string frame_id{"map"};
  double cell_size{kFallbackCellSize};
};

std::string trim_copy(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim_copy(field));
  }
  return fields;
}

double parse_finite_double(const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const double result = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(name + " must be a finite number: " + value);
  }
  return result;
}

std::map<std::string, std::size_t> csv_header_index(const std::vector<std::string> & header)
{
  std::map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < header.size(); ++i) {
    index.emplace(header[i], i);
  }
  return index;
}

std::size_t require_column(
  const std::map<std::string, std::size_t> & header_index, const std::string & name)
{
  const auto iter = header_index.find(name);
  if (iter == header_index.end()) {
    throw std::runtime_error("global_elevation_map.csv is missing required column: " + name);
  }
  return iter->second;
}

std::string unquote_yaml_scalar(std::string value)
{
  value = trim_copy(value);
  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
    (value.front() == '\'' && value.back() == '\'')))
  {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

ManifestInfo read_manifest(const std::filesystem::path & manifest_path)
{
  std::ifstream input(manifest_path);
  if (!input) {
    throw std::runtime_error("failed to open elevation manifest: " + manifest_path.string());
  }

  ManifestInfo info;
  bool in_parameters = false;
  bool has_cell_size = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment_position = line.find('#');
    if (comment_position != std::string::npos) {
      line = line.substr(0, comment_position);
    }
    if (trim_copy(line).empty()) {
      continue;
    }

    // manifestは単純なYAMLなので、frame_idと表示セルサイズだけを軽量に抽出する。
    const bool indented = !line.empty() && (line.front() == ' ' || line.front() == '\t');
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const auto key = trim_copy(line.substr(0, separator));
    const auto value = unquote_yaml_scalar(line.substr(separator + 1));
    if (!indented) {
      in_parameters = key == "parameters";
      if (key == "frame_id" && !value.empty()) {
        info.frame_id = value;
      }
      continue;
    }
    if (in_parameters && key == "elevation_output_cell_size" && !value.empty()) {
      info.cell_size = parse_finite_double(value, "elevation_output_cell_size");
      has_cell_size = true;
      continue;
    }
    if (in_parameters && key == "elevation_cell_size" && !value.empty()) {
      info.cell_size = parse_finite_double(value, "elevation_cell_size");
      has_cell_size = true;
    }
  }

  if (!has_cell_size || !std::isfinite(info.cell_size) || info.cell_size <= 0.0) {
    throw std::runtime_error(
      "elevation_manifest.yaml must contain positive parameters.elevation_output_cell_size");
  }
  return info;
}

}  // namespace

class ElevationMapMarkerViewerNode : public rclcpp::Node
{
public:
  ElevationMapMarkerViewerNode()
  : Node("elevation_map_marker_viewer_node")
  {
    map_dir_ = declare_parameter<std::string>("map_dir", "");
    if (map_dir_.empty()) {
      throw std::invalid_argument("map_dir parameter is required");
    }

    load_inputs();
    point_cloud_ = build_point_cloud();

    // RVizやtopic echoを後から起動しても受け取れるよう、保存済みmap表示はtransient localで保持する。
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      kPointCloudTopic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(kPublishPeriodSec),
      [this]() {publish_point_cloud();});
    publish_point_cloud();
  }

private:
  void load_inputs()
  {
    const std::filesystem::path map_dir(map_dir_);
    if (!std::filesystem::is_directory(map_dir)) {
      throw std::runtime_error("map_dir is not a directory: " + map_dir.string());
    }

    const auto csv_path = map_dir / kCsvFileName;
    const auto manifest_path = map_dir / kManifestFileName;
    manifest_ = read_manifest(manifest_path);
    cells_ = read_csv(csv_path);
    if (cells_.empty()) {
      throw std::runtime_error("global_elevation_map.csv has no data rows: " + csv_path.string());
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded elevation map: map_dir=%s frame_id=%s cell_size=%.4f points=%zu topic=%s",
      map_dir.string().c_str(), manifest_.frame_id.c_str(), manifest_.cell_size, cells_.size(), kPointCloudTopic);
  }

  std::vector<ElevationCell> read_csv(const std::filesystem::path & csv_path) const
  {
    std::ifstream input(csv_path);
    if (!input) {
      throw std::runtime_error("failed to open global elevation CSV: " + csv_path.string());
    }

    std::string line;
    if (!std::getline(input, line)) {
      throw std::runtime_error("global_elevation_map.csv is empty: " + csv_path.string());
    }
    const auto header = split_csv_line(line);
    const auto index = csv_header_index(header);
    const auto x_column = require_column(index, "x");
    const auto y_column = require_column(index, "y");
    const auto z_max_column = require_column(index, "z_max");

    std::vector<ElevationCell> cells;
    std::uint64_t row_index = 1;
    while (std::getline(input, line)) {
      ++row_index;
      if (trim_copy(line).empty()) {
        continue;
      }

      // PointCloud2表示ではCSVのセル中心とz_maxだけを使い、intensityにもz_maxを入れる。
      const auto fields = split_csv_line(line);
      const auto required_size = std::max({x_column, y_column, z_max_column}) + 1U;
      if (fields.size() < required_size) {
        throw std::runtime_error("CSV row has fewer columns than header at row " + std::to_string(row_index));
      }
      cells.push_back(ElevationCell{
        parse_finite_double(fields[x_column], "x"),
        parse_finite_double(fields[y_column], "y"),
        parse_finite_double(fields[z_max_column], "z_max")});
    }
    return cells;
  }

  sensor_msgs::msg::PointCloud2 build_point_cloud() const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = manifest_.frame_id;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cells_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    for (const auto & cell : cells_) {
      *iter_x = static_cast<float>(cell.x);
      *iter_y = static_cast<float>(cell.y);
      *iter_z = static_cast<float>(cell.z_max);
      *iter_intensity = static_cast<float>(cell.z_max);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
    return cloud;
  }

  void publish_point_cloud()
  {
    point_cloud_.header.stamp = get_clock()->now();
    publisher_->publish(point_cloud_);
  }

  std::string map_dir_;
  ManifestInfo manifest_;
  std::vector<ElevationCell> cells_;
  sensor_msgs::msg::PointCloud2 point_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ai_ship_robot_slam::ElevationMapMarkerViewerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("elevation_map_marker_viewer_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
