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

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ai_ship_robot_slam
{
namespace
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

constexpr const char * kLocalizationMapName = "localization_map.pcd";
constexpr const char * kDetailCloudName = "trajectory_seed_ring_cloud.pcd";
constexpr const char * kGroundCoarseCloudName = "ground_coarse_cloud.pcd";
constexpr const char * kGroundCoarsePgmName = "ground_coarse_map.pgm";
constexpr const char * kGroundCoarsePngName = "ground_coarse_map.png";
constexpr const char * kGroundCoarseYamlName = "ground_coarse_map.yaml";
constexpr const char * kGroundCandidateCloudName = "ground_candidate_cloud.pcd";
constexpr const char * kGroundCandidatePgmName = "ground_candidate_map.pgm";
constexpr const char * kGroundCandidatePngName = "ground_candidate_map.png";
constexpr const char * kGroundCandidateYamlName = "ground_candidate_map.yaml";
constexpr const char * kGroundTraversableCloudName = "ground_traversable_cloud.pcd";
constexpr const char * kGroundTraversablePgmName = "ground_traversable_map.pgm";
constexpr const char * kGroundTraversablePngName = "ground_traversable_map.png";
constexpr const char * kGroundTraversableYamlName = "ground_traversable_map.yaml";
constexpr const char * kGroundRefineTargetPgmName = "ground_refine_target_map.pgm";
constexpr const char * kGroundRefineTargetPngName = "ground_refine_target_map.png";
constexpr const char * kGroundRefineTargetYamlName = "ground_refine_target_map.yaml";
constexpr const char * kGroundRefineReasonPngName = "ground_refine_target_reason_map.png";
constexpr const char * kGroundTraversableReasonPngName = "ground_traversable_reason_map.png";
constexpr double kPi = 3.14159265358979323846;

enum class CellState : std::uint8_t
{
  Unknown,
  Candidate,
  Seed,
  Ground,
  RefineTarget,
  RejectedDensity,
  RejectedHeightRange,
  RejectedSlope,
  RejectedRoughness,
  RejectedNormalDiff,
  RejectedHeightGap,
  Disconnected,
};

enum class RefineReason : std::uint8_t
{
  None,
  DensityContrast,
  NonGroundCoarse,
  NoCoarsePlane,
  LowCoarseDensity,
  PlaneDistance,
  CoarseHeightRange,
};

struct Options
{
  std::filesystem::path map_dir;
  std::filesystem::path localization_path;
  std::filesystem::path detail_path;
  std::filesystem::path output_dir;
  std::filesystem::path config_path;
  double coarse_resolution{0.20};
  double coarse_connection_z_window{0.25};
  std::size_t coarse_plane_fit_radius{1};
  std::size_t coarse_min_plane_points{8};
  std::size_t coarse_origin_seed_radius_cells{1};
  double coarse_max_slope_deg{30.0};
  double coarse_max_normal_diff_deg{30.0};
  double coarse_max_neighbor_height_gap{0.20};
  double coarse_max_plane_rmse{0.10};
  bool write_debug_png{false};
  bool refinement_enabled{true};
  double refinement_resolution{0.05};
  double refinement_coarse_min_density{40.0};
  bool refinement_coarse_density_contrast_enabled{true};
  double refinement_coarse_density_contrast_ratio{0.35};
  double refinement_coarse_density_contrast_min_high_density{50.0};
  double refinement_min_point_height_below_ground{0.10};
  double refinement_max_point_height_above_ground{0.20};
  double refinement_coarse_max_plane_distance_p95{0.05};
  double refinement_coarse_max_height_range_p95{0.08};
  double refinement_percentile_low{5.0};
  double refinement_percentile_high{95.0};
  double refinement_min_density{20.0};
  std::size_t refinement_plane_fit_radius_cells{1};
  std::size_t refinement_min_plane_points{5};
  double refinement_max_height_range_p95{0.05};
  double refinement_max_slope_deg{25.0};
  double refinement_max_plane_rmse{0.05};
  double refinement_max_normal_diff_deg{35.0};
  double refinement_max_neighbor_height_gap{0.08};
};

struct Bounds
{
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};

struct Cell
{
  std::vector<std::size_t> point_indices;
  std::vector<std::size_t> ground_indices;
  bool enabled{true};
  bool refine_target{false};
  RefineReason refine_reason{RefineReason::None};
  bool plane_valid{false};
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double plane_rmse{std::numeric_limits<double>::infinity()};
  double slope_deg{std::numeric_limits<double>::infinity()};
  double representative_z{std::numeric_limits<double>::quiet_NaN()};
  CellState state{CellState::Unknown};
};

struct Grid
{
  std::size_t width{};
  std::size_t height{};
  double origin_x{};
  double origin_y{};
  double resolution{};
  std::vector<Cell> cells;
};

struct Stats
{
  std::size_t localization_points{};
  std::size_t detail_points{};
  std::size_t observed_cells{};
  std::size_t support_cells{};
  std::size_t plane_cells{};
  std::size_t seed_cells{};
  std::size_t ground_cells{};
  std::size_t refine_target_cells{};
  std::size_t rejected_cells{};
  std::size_t output_points{};
};

struct CoarseResult
{
  Grid grid;
  CloudT cloud;
};

struct RefinementResult
{
  Grid candidate_grid;
  Grid traversable_grid;
  Grid refine_target_grid;
  CloudT candidate_cloud;
  CloudT traversable_cloud;
  std::size_t refine_target_coarse_cells{};
  std::size_t refined_cells{};
  std::size_t density_rejections{};
  std::size_t height_rejections{};
  std::size_t slope_rejections{};
  std::size_t roughness_rejections{};
  std::size_t disconnected_cells{};
};

std::string take_option_value(
  const std::string & name, const std::optional<std::string> & inline_value, int & index,
  const int argc, char ** argv)
{
  if (inline_value.has_value()) {
    return *inline_value;
  }
  if (index + 1 >= argc) {
    throw std::invalid_argument(name + " requires a value");
  }
  ++index;
  return argv[index];
}

double parse_double(const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const auto result = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(name + " must be a finite number: " + value);
  }
  return result;
}

double parse_positive_double(const std::string & value, const std::string & name)
{
  const auto result = parse_double(value, name);
  if (result <= 0.0) {
    throw std::invalid_argument(name + " must be greater than zero: " + value);
  }
  return result;
}

double parse_nonnegative_double(const std::string & value, const std::string & name)
{
  const auto result = parse_double(value, name);
  if (result < 0.0) {
    throw std::invalid_argument(name + " must be non-negative: " + value);
  }
  return result;
}

std::size_t parse_size(const std::string & value, const std::string & name, const bool allow_zero)
{
  if (value.empty() || value.front() == '-') {
    throw std::invalid_argument(name + " must be a non-negative integer: " + value);
  }
  std::size_t parsed = 0;
  const auto result = std::stoull(value, &parsed);
  if (parsed != value.size() || result > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(name + " must be a non-negative integer: " + value);
  }
  if (!allow_zero && result == 0) {
    throw std::invalid_argument(name + " must be greater than zero: " + value);
  }
  return static_cast<std::size_t>(result);
}

bool parse_bool(const std::string & value, const std::string & name)
{
  std::string normalized;
  normalized.reserve(value.size());
  std::transform(value.begin(), value.end(), std::back_inserter(normalized), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }
  throw std::invalid_argument(name + " must be a boolean: " + value);
}

std::string trim_copy(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string strip_yaml_scalar(const std::string & text)
{
  auto value = trim_copy(text);
  const auto comment = value.find('#');
  if (comment != std::string::npos) {
    value = trim_copy(value.substr(0, comment));
  }
  if (value.size() >= 2 &&
    ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
  {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::string canonical_config_key(const std::string & key)
{
  auto canonical_key = key;
  const auto nested_root = canonical_key.find("groundCandidate.");
  if (nested_root != std::string::npos) {
    canonical_key = canonical_key.substr(nested_root);
  }

  if (canonical_key == "groundCandidate.coarseGround.gridResolution") {
    return "coarseResolution";
  }
  if (canonical_key == "groundCandidate.coarseGround.connectionZWindow") {
    return "coarseConnectionZWindow";
  }
  if (canonical_key == "groundCandidate.coarseGround.planeFitRadius") {
    return "coarsePlaneFitRadius";
  }
  if (canonical_key == "groundCandidate.coarseGround.minPlanePoints") {
    return "coarseMinPlanePoints";
  }
  if (canonical_key == "groundCandidate.coarseGround.originSeedRadiusCells") {
    return "coarseOriginSeedRadiusCells";
  }
  if (canonical_key == "groundCandidate.coarseGround.maxSlopeDeg") {
    return "coarseMaxSlopeDeg";
  }
  if (canonical_key == "groundCandidate.coarseGround.maxNormalDiffDeg") {
    return "coarseMaxNormalDiffDeg";
  }
  if (canonical_key == "groundCandidate.coarseGround.maxNeighborHeightGap") {
    return "coarseMaxNeighborHeightGap";
  }
  if (canonical_key == "groundCandidate.coarseGround.maxPlaneRmse") {
    return "coarseMaxPlaneRmse";
  }
  if (canonical_key == "groundCandidate.coarseGround.writeDebugPng") {
    return "writeDebugPng";
  }
  if (canonical_key == "groundCandidate.detailRefinement.enabled") {
    return "refinementEnabled";
  }
  if (canonical_key == "groundCandidate.detailRefinement.gridResolution") {
    return "refinementResolution";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseMinDensity") {
    return "refinementCoarseMinDensity";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseDensityContrastEnabled") {
    return "refinementCoarseDensityContrastEnabled";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseDensityContrastRatio") {
    return "refinementCoarseDensityContrastRatio";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseDensityContrastMinHighDensity") {
    return "refinementCoarseDensityContrastMinHighDensity";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseMaxPlaneDistanceP95") {
    return "refinementCoarseMaxPlaneDistanceP95";
  }
  if (canonical_key == "groundCandidate.detailRefinement.minPointHeightBelowGround") {
    return "refinementMinPointHeightBelowGround";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxPointHeightAboveGround") {
    return "refinementMaxPointHeightAboveGround";
  }
  if (canonical_key == "groundCandidate.detailRefinement.coarseMaxHeightRangeP95") {
    return "refinementCoarseMaxHeightRangeP95";
  }
  if (canonical_key == "groundCandidate.detailRefinement.percentileLow") {
    return "refinementPercentileLow";
  }
  if (canonical_key == "groundCandidate.detailRefinement.percentileHigh") {
    return "refinementPercentileHigh";
  }
  if (canonical_key == "groundCandidate.detailRefinement.minDensity") {
    return "refinementMinDensity";
  }
  if (canonical_key == "groundCandidate.detailRefinement.planeFitRadiusCells") {
    return "refinementPlaneFitRadiusCells";
  }
  if (canonical_key == "groundCandidate.detailRefinement.minPlanePoints") {
    return "refinementMinPlanePoints";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxHeightRangeP95") {
    return "refinementMaxHeightRangeP95";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxSlopeDeg") {
    return "refinementMaxSlopeDeg";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxPlaneRmse") {
    return "refinementMaxPlaneRmse";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxNormalDiffDeg") {
    return "refinementMaxNormalDiffDeg";
  }
  if (canonical_key == "groundCandidate.detailRefinement.maxNeighborHeightGap") {
    return "refinementMaxNeighborHeightGap";
  }
  return canonical_key;
}

void apply_config_value(Options & options, const std::string & key, const std::string & value)
{
  if (value.empty()) {
    return;
  }
  const auto canonical_key = canonical_config_key(key);

  // ROS parameter YAML内の地面地図生成キーだけを読み、LIO-SAM本体設定は無視する。
  if (canonical_key == "coarseResolution" || canonical_key == "groundGridResolution") {
    options.coarse_resolution = parse_positive_double(value, canonical_key);
  } else if (canonical_key == "coarseConnectionZWindow" || canonical_key == "groundConnectionZWindow") {
    options.coarse_connection_z_window = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "coarsePlaneFitRadius" || canonical_key == "groundPlaneFitRadius") {
    options.coarse_plane_fit_radius = parse_size(value, canonical_key, true);
  } else if (canonical_key == "coarseMinPlanePoints" || canonical_key == "groundMinPlanePoints") {
    options.coarse_min_plane_points = parse_size(value, canonical_key, false);
  } else if (canonical_key == "coarseOriginSeedRadiusCells" || canonical_key == "groundOriginSeedRadiusCells") {
    options.coarse_origin_seed_radius_cells = parse_size(value, canonical_key, true);
  } else if (canonical_key == "coarseMaxSlopeDeg" || canonical_key == "groundMaxSlopeDeg") {
    options.coarse_max_slope_deg = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "coarseMaxNormalDiffDeg" || canonical_key == "groundMaxNormalDiffDeg") {
    options.coarse_max_normal_diff_deg = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "coarseMaxNeighborHeightGap" || canonical_key == "groundMaxNeighborHeightGap") {
    options.coarse_max_neighbor_height_gap = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "coarseMaxPlaneRmse" || canonical_key == "groundMaxPlaneRmse") {
    options.coarse_max_plane_rmse = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "writeDebugPng" || canonical_key == "groundWriteDebugPng") {
    options.write_debug_png = parse_bool(value, canonical_key);
  } else if (canonical_key == "refinementEnabled") {
    options.refinement_enabled = parse_bool(value, canonical_key);
  } else if (canonical_key == "refinementResolution") {
    options.refinement_resolution = parse_positive_double(value, canonical_key);
  } else if (canonical_key == "refinementCoarseMinDensity") {
    options.refinement_coarse_min_density = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementCoarseDensityContrastEnabled") {
    options.refinement_coarse_density_contrast_enabled = parse_bool(value, canonical_key);
  } else if (canonical_key == "refinementCoarseDensityContrastRatio") {
    options.refinement_coarse_density_contrast_ratio = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementCoarseDensityContrastMinHighDensity") {
    options.refinement_coarse_density_contrast_min_high_density = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMinPointHeightBelowGround") {
    options.refinement_min_point_height_below_ground = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMaxPointHeightAboveGround") {
    options.refinement_max_point_height_above_ground = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementCoarseMaxPlaneDistanceP95") {
    options.refinement_coarse_max_plane_distance_p95 = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementCoarseMaxHeightRangeP95") {
    options.refinement_coarse_max_height_range_p95 = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementPercentileLow") {
    options.refinement_percentile_low = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementPercentileHigh") {
    options.refinement_percentile_high = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMinDensity") {
    options.refinement_min_density = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementPlaneFitRadiusCells") {
    options.refinement_plane_fit_radius_cells = parse_size(value, canonical_key, true);
  } else if (canonical_key == "refinementMinPlanePoints") {
    options.refinement_min_plane_points = parse_size(value, canonical_key, false);
  } else if (canonical_key == "refinementMaxHeightRangeP95") {
    options.refinement_max_height_range_p95 = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMaxSlopeDeg") {
    options.refinement_max_slope_deg = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMaxPlaneRmse") {
    options.refinement_max_plane_rmse = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMaxNormalDiffDeg") {
    options.refinement_max_normal_diff_deg = parse_nonnegative_double(value, canonical_key);
  } else if (canonical_key == "refinementMaxNeighborHeightGap") {
    options.refinement_max_neighbor_height_gap = parse_nonnegative_double(value, canonical_key);
  }
}

void load_config_file(Options & options, const std::filesystem::path & config_path)
{
  std::ifstream file(config_path);
  if (!file) {
    throw std::runtime_error("failed to open config YAML: " + config_path.string());
  }

  std::string line;
  std::vector<std::pair<std::size_t, std::string>> key_stack;
  while (std::getline(file, line)) {
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const auto indent = line.find_first_not_of(' ');
    if (indent == std::string::npos || line[indent] == '#') {
      continue;
    }
    const auto key = trim_copy(line.substr(0, separator));
    const auto value = strip_yaml_scalar(line.substr(separator + 1));
    while (!key_stack.empty() && key_stack.back().first >= indent) {
      key_stack.pop_back();
    }

    // ネストしたROS parameter YAMLをドット区切りへ変換し、設定分類ごとに読めるようにする。
    std::string full_key;
    for (const auto & [unused_indent, parent_key] : key_stack) {
      (void)unused_indent;
      if (!full_key.empty()) {
        full_key += '.';
      }
      full_key += parent_key;
    }
    if (!full_key.empty()) {
      full_key += '.';
    }
    full_key += key;

    apply_config_value(options, full_key, value);
    if (value.empty()) {
      key_stack.emplace_back(indent, key);
    }
  }
}

void print_usage(std::ostream & stream)
{
  stream << "Usage: ros2 run ai_ship_robot_slam ground_candidate_map_generator [OPTIONS]\n\n";
  stream << "Options:\n";
  stream << "  --map-dir DIR                    Directory containing localization/detail PCDs.\n";
  stream << "  --localization PATH              Input localization_map.pcd path.\n";
  stream << "  --seed PATH                      Input trajectory_seed_ring_cloud.pcd path.\n";
  stream << "  --output-dir DIR                 Output directory. Default: map-dir or localization parent.\n";
  stream << "  --config PATH                    YAML containing groundCandidate parameters.\n";
  stream << "  --resolution M                   Coarse grid resolution in meters. Default: 0.20\n";
  stream << "  --origin-seed-radius-cells N     Coarse seed radius around map origin. Default: 1\n";
  stream << "  -h, --help                       Show this help.\n";
}

std::filesystem::path find_workspace_root()
{
  if (const auto * workspace_root = std::getenv("AI_SHIP_ROBOT_WORKSPACE_ROOT")) {
    return std::filesystem::path(workspace_root);
  }

  // 任意の場所から実行されても、プロジェクト固有の配置でワークスペースrootを推定する。
  auto directory = std::filesystem::current_path();
  while (true) {
    const auto package_xml = directory / "ros2_ws" / "src" / "ai_ship_robot_slam" / "package.xml";
    if (std::filesystem::exists(package_xml)) {
      return directory;
    }
    if (directory == directory.root_path()) {
      break;
    }
    directory = directory.parent_path();
  }
  return std::filesystem::current_path();
}

std::filesystem::path find_latest_map_dir()
{
  const auto cloud_map_root = find_workspace_root() / "outputs" / "cloud_map";
  if (!std::filesystem::is_directory(cloud_map_root)) {
    throw std::runtime_error("cloud map output directory does not exist: " + cloud_map_root.string());
  }

  std::filesystem::path best_dir;
  std::filesystem::file_time_type best_time{};
  for (const auto & entry : std::filesystem::directory_iterator(cloud_map_root)) {
    if (!entry.is_directory()) {
      continue;
    }
    const auto candidate_dir = entry.path();
    if (!std::filesystem::exists(candidate_dir / kLocalizationMapName) ||
      !std::filesystem::exists(candidate_dir / kDetailCloudName))
    {
      continue;
    }
    std::error_code error;
    const auto modified_time = std::filesystem::last_write_time(candidate_dir / kLocalizationMapName, error);
    if (error) {
      continue;
    }
    if (best_dir.empty() || modified_time > best_time) {
      best_dir = candidate_dir;
      best_time = modified_time;
    }
  }
  if (best_dir.empty()) {
    throw std::runtime_error("no map directory with required PCD files was found under: " + cloud_map_root.string());
  }
  return best_dir;
}

void validate_options(const Options & options)
{
  if (options.localization_path.empty() || !std::filesystem::exists(options.localization_path)) {
    throw std::invalid_argument("localization PCD does not exist: " + options.localization_path.string());
  }
  if (options.detail_path.empty() || !std::filesystem::exists(options.detail_path)) {
    throw std::invalid_argument("detail PCD does not exist: " + options.detail_path.string());
  }
  if (!options.config_path.empty() && !std::filesystem::exists(options.config_path)) {
    throw std::invalid_argument("config YAML does not exist: " + options.config_path.string());
  }
  if (options.output_dir.empty()) {
    throw std::invalid_argument("output directory is empty");
  }
  if (options.refinement_percentile_low < 0.0 || options.refinement_percentile_high > 100.0 ||
    options.refinement_percentile_low >= options.refinement_percentile_high)
  {
    throw std::invalid_argument("detailRefinement percentileLow/High must satisfy 0 <= low < high <= 100");
  }
}

void resolve_options(Options & options)
{
  if (options.map_dir.empty() && options.localization_path.empty() && options.detail_path.empty()) {
    options.map_dir = find_latest_map_dir();
  }
  if (!options.map_dir.empty()) {
    if (options.localization_path.empty()) {
      options.localization_path = options.map_dir / kLocalizationMapName;
    }
    if (options.detail_path.empty()) {
      options.detail_path = options.map_dir / kDetailCloudName;
    }
    if (options.output_dir.empty()) {
      options.output_dir = options.map_dir;
    }
  }
  if (options.output_dir.empty() && !options.localization_path.empty()) {
    options.output_dir = options.localization_path.parent_path();
  }
  validate_options(options);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    std::string argument = argv[index];
    std::optional<std::string> inline_value;
    const auto separator = argument.find('=');
    if (separator != std::string::npos) {
      inline_value = argument.substr(separator + 1);
      argument = argument.substr(0, separator);
    }

    if (argument == "-h" || argument == "--help") {
      print_usage(std::cout);
      std::exit(0);
    } else if (argument == "--map-dir") {
      options.map_dir = take_option_value(argument, inline_value, index, argc, argv);
    } else if (argument == "--localization") {
      options.localization_path = take_option_value(argument, inline_value, index, argc, argv);
    } else if (argument == "--seed") {
      options.detail_path = take_option_value(argument, inline_value, index, argc, argv);
    } else if (argument == "--output-dir") {
      options.output_dir = take_option_value(argument, inline_value, index, argc, argv);
    } else if (argument == "--config") {
      options.config_path = take_option_value(argument, inline_value, index, argc, argv);
      load_config_file(options, options.config_path);
    } else if (argument == "--resolution") {
      options.coarse_resolution = parse_positive_double(take_option_value(argument, inline_value, index, argc, argv), argument);
    } else if (argument == "--origin-seed-radius-cells") {
      options.coarse_origin_seed_radius_cells = parse_size(
        take_option_value(argument, inline_value, index, argc, argv), argument, true);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  resolve_options(options);
  return options;
}

CloudT::Ptr load_pcd(const std::filesystem::path & path)
{
  auto cloud = std::make_shared<CloudT>();
  if (pcl::io::loadPCDFile<PointT>(path.string(), *cloud) != 0) {
    throw std::runtime_error("failed to load PCD: " + path.string());
  }
  if (cloud->empty()) {
    throw std::runtime_error("PCD is empty: " + path.string());
  }
  return cloud;
}

Eigen::Vector3d to_vector3d(const PointT & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

std::size_t flatten_index(const std::size_t x, const std::size_t y, const std::size_t width)
{
  return y * width + x;
}

bool is_ground_state(const CellState state)
{
  return state == CellState::Seed || state == CellState::Ground;
}

bool is_rejected_state(const CellState state)
{
  return state == CellState::RejectedDensity || state == CellState::RejectedHeightRange ||
         state == CellState::RejectedSlope || state == CellState::RejectedRoughness ||
         state == CellState::RejectedNormalDiff || state == CellState::RejectedHeightGap ||
         state == CellState::Disconnected;
}

Bounds compute_bounds(const CloudT & cloud)
{
  Bounds bounds;
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    bounds.min_x = std::min(bounds.min_x, static_cast<double>(point.x));
    bounds.min_y = std::min(bounds.min_y, static_cast<double>(point.y));
    bounds.max_x = std::max(bounds.max_x, static_cast<double>(point.x));
    bounds.max_y = std::max(bounds.max_y, static_cast<double>(point.y));
  }
  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.max_x) || !std::isfinite(bounds.min_y) ||
    !std::isfinite(bounds.max_y))
  {
    throw std::runtime_error("cloud does not contain finite XY points");
  }
  return bounds;
}

Grid create_empty_grid(const Bounds & bounds, const double resolution)
{
  Grid grid;
  grid.resolution = resolution;
  grid.origin_x = std::floor(bounds.min_x / resolution) * resolution;
  grid.origin_y = std::floor(bounds.min_y / resolution) * resolution;
  const auto max_x = std::ceil(bounds.max_x / resolution) * resolution;
  const auto max_y = std::ceil(bounds.max_y / resolution) * resolution;
  grid.width = static_cast<std::size_t>(std::ceil((max_x - grid.origin_x) / resolution)) + 1U;
  grid.height = static_cast<std::size_t>(std::ceil((max_y - grid.origin_y) / resolution)) + 1U;
  grid.cells.resize(grid.width * grid.height);
  return grid;
}

bool locate_cell(const Grid & grid, const double x, const double y, std::size_t & cell_x, std::size_t & cell_y)
{
  const auto relative_x = (x - grid.origin_x) / grid.resolution;
  const auto relative_y = (y - grid.origin_y) / grid.resolution;
  if (relative_x < 0.0 || relative_y < 0.0) {
    return false;
  }
  const auto signed_x = static_cast<std::int64_t>(std::floor(relative_x));
  const auto signed_y = static_cast<std::int64_t>(std::floor(relative_y));
  if (signed_x < 0 || signed_y < 0 || signed_x >= static_cast<std::int64_t>(grid.width) ||
    signed_y >= static_cast<std::int64_t>(grid.height))
  {
    return false;
  }
  cell_x = static_cast<std::size_t>(signed_x);
  cell_y = static_cast<std::size_t>(signed_y);
  return true;
}

void assign_points(Grid & grid, const CloudT & cloud)
{
  for (std::size_t point_index = 0; point_index < cloud.points.size(); ++point_index) {
    const auto & point = cloud.points[point_index];
    std::size_t cell_x = 0;
    std::size_t cell_y = 0;
    if (!locate_cell(grid, point.x, point.y, cell_x, cell_y)) {
      continue;
    }
    grid.cells[flatten_index(cell_x, cell_y, grid.width)].point_indices.push_back(point_index);
  }
}

Grid build_grid(const Bounds & bounds, const CloudT & cloud, const double resolution)
{
  auto grid = create_empty_grid(bounds, resolution);
  assign_points(grid, cloud);
  return grid;
}

std::pair<double, double> cell_center(const Grid & grid, const std::size_t cell_x, const std::size_t cell_y)
{
  return {grid.origin_x + (static_cast<double>(cell_x) + 0.5) * grid.resolution,
          grid.origin_y + (static_cast<double>(cell_y) + 0.5) * grid.resolution};
}

double percentile_from_sorted(const std::vector<double> & sorted_values, const double percentile)
{
  if (sorted_values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto clamped = std::clamp(percentile, 0.0, 100.0) / 100.0;
  const auto position = clamped * static_cast<double>(sorted_values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return sorted_values[lower];
  }
  const auto ratio = position - static_cast<double>(lower);
  return sorted_values[lower] * (1.0 - ratio) + sorted_values[upper] * ratio;
}

double percentile(std::vector<double> values, const double percentile_value)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::sort(values.begin(), values.end());
  return percentile_from_sorted(values, percentile_value);
}

double percentile_height_range(
  const CloudT & cloud, const std::vector<std::size_t> & point_indices, const double low_percentile,
  const double high_percentile)
{
  std::vector<double> z_values;
  z_values.reserve(point_indices.size());
  for (const auto point_index : point_indices) {
    z_values.push_back(cloud.points[point_index].z);
  }
  if (z_values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::sort(z_values.begin(), z_values.end());
  return percentile_from_sorted(z_values, high_percentile) - percentile_from_sorted(z_values, low_percentile);
}

double density_contrast_ratio_in_coarse_cell(
  const Grid & coarse_grid, const CloudT & cloud, const Cell & coarse_cell, const std::size_t coarse_x,
  const std::size_t coarse_y, const Options & options, double & high_density)
{
  const auto sub_divisions = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(coarse_grid.resolution / options.refinement_resolution)));
  std::vector<double> counts(sub_divisions * sub_divisions, 0.0);
  const auto coarse_min_x = coarse_grid.origin_x + static_cast<double>(coarse_x) * coarse_grid.resolution;
  const auto coarse_min_y = coarse_grid.origin_y + static_cast<double>(coarse_y) * coarse_grid.resolution;
  const auto subcell_area = options.refinement_resolution * options.refinement_resolution;

  for (const auto point_index : coarse_cell.point_indices) {
    const auto & point = cloud.points[point_index];
    const auto sub_x = static_cast<std::int64_t>(std::floor((point.x - coarse_min_x) / options.refinement_resolution));
    const auto sub_y = static_cast<std::int64_t>(std::floor((point.y - coarse_min_y) / options.refinement_resolution));
    if (sub_x < 0 || sub_y < 0 || sub_x >= static_cast<std::int64_t>(sub_divisions) ||
      sub_y >= static_cast<std::int64_t>(sub_divisions))
    {
      continue;
    }
    counts[flatten_index(static_cast<std::size_t>(sub_x), static_cast<std::size_t>(sub_y), sub_divisions)] += 1.0;
  }

  std::vector<double> densities;
  densities.reserve(counts.size());
  for (const auto count : counts) {
    densities.push_back(count / subcell_area);
  }
  std::sort(densities.begin(), densities.end());
  high_density = percentile_from_sorted(densities, options.refinement_percentile_high);
  if (high_density <= 0.0) {
    return 1.0;
  }
  const auto low_density = percentile_from_sorted(densities, options.refinement_percentile_low);
  return low_density / high_density;
}

double mean_z(const CloudT & cloud, const std::vector<std::size_t> & point_indices)
{
  if (point_indices.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double z_sum = 0.0;
  for (const auto point_index : point_indices) {
    z_sum += cloud.points[point_index].z;
  }
  return z_sum / static_cast<double>(point_indices.size());
}

bool fit_plane_from_indices(
  const CloudT & cloud, const std::vector<std::size_t> & point_indices, const std::size_t min_points,
  Eigen::Vector3d & centroid, Eigen::Vector3d & normal, double & rmse, double & slope_deg)
{
  if (point_indices.size() < min_points) {
    return false;
  }

  // PCAの最小固有値方向を地面候補平面の法線として使い、平面粗さをRMSEで評価する。
  centroid = Eigen::Vector3d::Zero();
  for (const auto point_index : point_indices) {
    centroid += to_vector3d(cloud.points[point_index]);
  }
  centroid /= static_cast<double>(point_indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto point_index : point_indices) {
    const auto centered = to_vector3d(cloud.points[point_index]) - centroid;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(point_indices.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  normal = solver.eigenvectors().col(0).normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  if (std::abs(normal.z()) < 1.0e-6) {
    return false;
  }
  rmse = std::sqrt(std::max(0.0, solver.eigenvalues()(0)));
  slope_deg = std::acos(std::clamp(normal.z(), -1.0, 1.0)) * 180.0 / kPi;
  return std::isfinite(rmse) && std::isfinite(slope_deg);
}

double plane_z_at(const Cell & cell, const double x, const double y)
{
  if (!cell.plane_valid || std::abs(cell.normal.z()) < 1.0e-6) {
    return std::numeric_limits<double>::infinity();
  }
  const auto dx = x - cell.centroid.x();
  const auto dy = y - cell.centroid.y();
  return cell.centroid.z() - (cell.normal.x() * dx + cell.normal.y() * dy) / cell.normal.z();
}

double point_plane_distance(const Cell & cell, const PointT & point)
{
  return std::abs(cell.normal.dot(to_vector3d(point) - cell.centroid));
}

bool is_point_in_ground_height_band(const Cell & ground_cell, const PointT & point, const Options & options)
{
  const auto ground_z = plane_z_at(ground_cell, point.x, point.y);
  if (!std::isfinite(ground_z)) {
    return false;
  }
  const auto dz = static_cast<double>(point.z) - ground_z;
  return dz >= -options.refinement_min_point_height_below_ground &&
         dz <= options.refinement_max_point_height_above_ground;
}

std::vector<std::size_t> filter_points_in_ground_height_band(
  const Cell & cell, const Cell & ground_cell, const CloudT & cloud, const Options & options)
{
  std::vector<std::size_t> selected;
  selected.reserve(cell.point_indices.size());
  for (const auto point_index : cell.point_indices) {
    if (is_point_in_ground_height_band(ground_cell, cloud.points[point_index], options)) {
      selected.push_back(point_index);
    }
  }
  return selected;
}

double normal_angle_deg(const Eigen::Vector3d & left, const Eigen::Vector3d & right)
{
  return std::acos(std::clamp(left.dot(right), -1.0, 1.0)) * 180.0 / kPi;
}

std::vector<std::size_t> select_cell_points_near_plane(
  const Cell & cell, const CloudT & cloud, const Cell & reference_cell, const double z_window)
{
  std::vector<std::size_t> selected;
  selected.reserve(cell.point_indices.size());
  for (const auto point_index : cell.point_indices) {
    const auto & point = cloud.points[point_index];
    const auto predicted_z = plane_z_at(reference_cell, point.x, point.y);
    if (std::isfinite(predicted_z) && std::abs(static_cast<double>(point.z) - predicted_z) <= z_window) {
      selected.push_back(point_index);
    }
  }
  return selected;
}

std::vector<std::size_t> collect_neighbor_points_near_plane(
  const Grid & grid, const CloudT & cloud, const std::size_t cell_x, const std::size_t cell_y,
  const Cell & reference_cell, const std::size_t radius_cells, const double z_window)
{
  std::vector<std::size_t> selected;
  const auto radius = static_cast<std::int64_t>(radius_cells);
  for (std::int64_t offset_y = -radius; offset_y <= radius; ++offset_y) {
    const auto neighbor_y = static_cast<std::int64_t>(cell_y) + offset_y;
    if (neighbor_y < 0 || neighbor_y >= static_cast<std::int64_t>(grid.height)) {
      continue;
    }
    for (std::int64_t offset_x = -radius; offset_x <= radius; ++offset_x) {
      const auto neighbor_x = static_cast<std::int64_t>(cell_x) + offset_x;
      if (neighbor_x < 0 || neighbor_x >= static_cast<std::int64_t>(grid.width)) {
        continue;
      }
      const auto & neighbor = grid.cells[flatten_index(
        static_cast<std::size_t>(neighbor_x), static_cast<std::size_t>(neighbor_y), grid.width)];
      const auto neighbor_points = select_cell_points_near_plane(neighbor, cloud, reference_cell, z_window);
      selected.insert(selected.end(), neighbor_points.begin(), neighbor_points.end());
    }
  }
  return selected;
}

std::vector<std::size_t> collect_neighbor_points(
  const Grid & grid, const std::size_t cell_x, const std::size_t cell_y, const std::size_t radius_cells)
{
  std::vector<std::size_t> selected;
  const auto radius = static_cast<std::int64_t>(radius_cells);
  for (std::int64_t offset_y = -radius; offset_y <= radius; ++offset_y) {
    const auto neighbor_y = static_cast<std::int64_t>(cell_y) + offset_y;
    if (neighbor_y < 0 || neighbor_y >= static_cast<std::int64_t>(grid.height)) {
      continue;
    }
    for (std::int64_t offset_x = -radius; offset_x <= radius; ++offset_x) {
      const auto neighbor_x = static_cast<std::int64_t>(cell_x) + offset_x;
      if (neighbor_x < 0 || neighbor_x >= static_cast<std::int64_t>(grid.width)) {
        continue;
      }
      const auto & neighbor = grid.cells[flatten_index(
        static_cast<std::size_t>(neighbor_x), static_cast<std::size_t>(neighbor_y), grid.width)];
      if (neighbor.state != CellState::Candidate && !is_ground_state(neighbor.state)) {
        continue;
      }
      selected.insert(selected.end(), neighbor.point_indices.begin(), neighbor.point_indices.end());
    }
  }
  return selected;
}

double boundary_height_gap(
  const Grid & grid, const std::size_t current_x, const std::size_t current_y, const Cell & current,
  const std::size_t neighbor_x, const std::size_t neighbor_y, const Cell & neighbor)
{
  const auto [current_center_x, current_center_y] = cell_center(grid, current_x, current_y);
  const auto [neighbor_center_x, neighbor_center_y] = cell_center(grid, neighbor_x, neighbor_y);
  const auto boundary_x = 0.5 * (current_center_x + neighbor_center_x);
  const auto boundary_y = 0.5 * (current_center_y + neighbor_center_y);
  const auto current_z = plane_z_at(current, boundary_x, boundary_y);
  const auto neighbor_z = plane_z_at(neighbor, boundary_x, boundary_y);
  return std::abs(current_z - neighbor_z);
}

void initialize_candidate_cells(Grid & grid)
{
  for (auto & cell : grid.cells) {
    cell.state = cell.point_indices.empty() ? CellState::Unknown : CellState::Candidate;
  }
}

void mark_origin_seed_cells(Grid & grid, const CloudT & cloud, const Options & options, std::deque<std::size_t> & queue)
{
  std::size_t origin_x = 0;
  std::size_t origin_y = 0;
  if (!locate_cell(grid, 0.0, 0.0, origin_x, origin_y)) {
    throw std::runtime_error("map grid does not contain origin for coarse ground seed");
  }

  const auto radius = static_cast<std::int64_t>(options.coarse_origin_seed_radius_cells);
  for (std::int64_t offset_y = -radius; offset_y <= radius; ++offset_y) {
    const auto y_signed = static_cast<std::int64_t>(origin_y) + offset_y;
    if (y_signed < 0 || y_signed >= static_cast<std::int64_t>(grid.height)) {
      continue;
    }
    for (std::int64_t offset_x = -radius; offset_x <= radius; ++offset_x) {
      const auto x_signed = static_cast<std::int64_t>(origin_x) + offset_x;
      if (x_signed < 0 || x_signed >= static_cast<std::int64_t>(grid.width)) {
        continue;
      }
      const auto cell_index = flatten_index(
        static_cast<std::size_t>(x_signed), static_cast<std::size_t>(y_signed), grid.width);
      auto & cell = grid.cells[cell_index];
      if (cell.point_indices.empty()) {
        continue;
      }

      // 原点近傍は平坦床と仮定し、セル内の低いz帯を初期地面としてregion growingを始める。
      auto min_z = std::numeric_limits<double>::infinity();
      for (const auto point_index : cell.point_indices) {
        min_z = std::min(min_z, static_cast<double>(cloud.points[point_index].z));
      }
      for (const auto point_index : cell.point_indices) {
        if (std::abs(static_cast<double>(cloud.points[point_index].z) - min_z) <= options.coarse_connection_z_window) {
          cell.ground_indices.push_back(point_index);
        }
      }
      if (cell.ground_indices.empty()) {
        continue;
      }
      cell.centroid = Eigen::Vector3d::Zero();
      for (const auto point_index : cell.ground_indices) {
        cell.centroid += to_vector3d(cloud.points[point_index]);
      }
      cell.centroid /= static_cast<double>(cell.ground_indices.size());
      cell.normal = Eigen::Vector3d::UnitZ();
      cell.plane_rmse = 0.0;
      cell.slope_deg = 0.0;
      cell.representative_z = mean_z(cloud, cell.ground_indices);
      cell.plane_valid = true;
      cell.state = CellState::Seed;
      queue.push_back(cell_index);
    }
  }
}

void grow_coarse_ground(Grid & grid, const CloudT & cloud, const Options & options)
{
  std::deque<std::size_t> queue;
  mark_origin_seed_cells(grid, cloud, options, queue);
  std::vector<std::uint8_t> accepted(grid.cells.size(), 0);
  for (const auto cell_index : queue) {
    accepted[cell_index] = 1;
  }

  static constexpr std::array<std::pair<int, int>, 8> kNeighborOffsets{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1},
  }};

  while (!queue.empty()) {
    const auto current_index = queue.front();
    queue.pop_front();
    const auto current_x = current_index % grid.width;
    const auto current_y = current_index / grid.width;
    const auto & current = grid.cells[current_index];

    // 到達済みの粗い地面平面に近い点だけを支持点として、隣接セルへ地面連結を広げる。
    for (const auto & [offset_x, offset_y] : kNeighborOffsets) {
      const auto neighbor_x_signed = static_cast<std::int64_t>(current_x) + offset_x;
      const auto neighbor_y_signed = static_cast<std::int64_t>(current_y) + offset_y;
      if (neighbor_x_signed < 0 || neighbor_y_signed < 0 ||
        neighbor_x_signed >= static_cast<std::int64_t>(grid.width) ||
        neighbor_y_signed >= static_cast<std::int64_t>(grid.height))
      {
        continue;
      }
      const auto neighbor_x = static_cast<std::size_t>(neighbor_x_signed);
      const auto neighbor_y = static_cast<std::size_t>(neighbor_y_signed);
      const auto neighbor_index = flatten_index(neighbor_x, neighbor_y, grid.width);
      if (accepted[neighbor_index] != 0) {
        continue;
      }
      auto & neighbor = grid.cells[neighbor_index];
      if (neighbor.point_indices.empty()) {
        continue;
      }

      auto cell_support = select_cell_points_near_plane(neighbor, cloud, current, options.coarse_connection_z_window);
      if (cell_support.empty()) {
        neighbor.state = CellState::RejectedRoughness;
        continue;
      }
      const auto fit_support = collect_neighbor_points_near_plane(
        grid, cloud, neighbor_x, neighbor_y, current, options.coarse_plane_fit_radius,
        options.coarse_connection_z_window);
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
      double rmse = std::numeric_limits<double>::infinity();
      double slope_deg = std::numeric_limits<double>::infinity();
      if (!fit_plane_from_indices(
          cloud, fit_support, options.coarse_min_plane_points, centroid, normal, rmse, slope_deg))
      {
        neighbor.state = CellState::RejectedRoughness;
        continue;
      }
      neighbor.ground_indices = std::move(cell_support);
      neighbor.centroid = centroid;
      neighbor.normal = normal;
      neighbor.plane_rmse = rmse;
      neighbor.slope_deg = slope_deg;
      neighbor.representative_z = mean_z(cloud, neighbor.ground_indices);
      neighbor.plane_valid = true;

      if (neighbor.plane_rmse > options.coarse_max_plane_rmse) {
        neighbor.state = CellState::RejectedRoughness;
        continue;
      }
      if (neighbor.slope_deg > options.coarse_max_slope_deg) {
        neighbor.state = CellState::RejectedSlope;
        continue;
      }
      if (normal_angle_deg(current.normal, neighbor.normal) > options.coarse_max_normal_diff_deg) {
        neighbor.state = CellState::RejectedNormalDiff;
        continue;
      }
      if (boundary_height_gap(grid, current_x, current_y, current, neighbor_x, neighbor_y, neighbor) >
        options.coarse_max_neighbor_height_gap)
      {
        neighbor.state = CellState::RejectedHeightGap;
        continue;
      }
      neighbor.state = CellState::Ground;
      accepted[neighbor_index] = 1;
      queue.push_back(neighbor_index);
    }
  }
}

CloudT build_cloud_from_ground_cells(const Grid & grid, const CloudT & source_cloud)
{
  CloudT output;
  for (const auto & cell : grid.cells) {
    if (!is_ground_state(cell.state)) {
      continue;
    }
    const auto & indices = cell.ground_indices.empty() ? cell.point_indices : cell.ground_indices;
    for (const auto point_index : indices) {
      output.push_back(source_cloud.points[point_index]);
    }
  }
  output.width = static_cast<std::uint32_t>(output.size());
  output.height = 1;
  output.is_dense = false;
  return output;
}

CoarseResult run_coarse_detection(const CloudT & localization_cloud, const Bounds & bounds, const Options & options)
{
  auto grid = build_grid(bounds, localization_cloud, options.coarse_resolution);
  initialize_candidate_cells(grid);
  grow_coarse_ground(grid, localization_cloud, options);
  auto cloud = build_cloud_from_ground_cells(grid, localization_cloud);
  return CoarseResult{std::move(grid), std::move(cloud)};
}

RefineReason classify_refine_coarse_cell(
  const Grid & coarse_grid, const Cell & coarse_cell, const Cell & measurement_cell, const CloudT & cloud,
  const std::size_t cell_x, const std::size_t cell_y, const Options & options)
{
  if (measurement_cell.point_indices.empty()) {
    return RefineReason::None;
  }

  double high_density = 0.0;
  const auto density_contrast = density_contrast_ratio_in_coarse_cell(
    coarse_grid, cloud, measurement_cell, cell_x, cell_y, options, high_density);
  if (options.refinement_coarse_density_contrast_enabled &&
    high_density >= options.refinement_coarse_density_contrast_min_high_density &&
    density_contrast < options.refinement_coarse_density_contrast_ratio)
  {
    return RefineReason::DensityContrast;
  }

  if (!is_ground_state(coarse_cell.state)) {
    return RefineReason::NonGroundCoarse;
  }
  if (!coarse_cell.plane_valid) {
    return RefineReason::NoCoarsePlane;
  }
  const auto band_points = filter_points_in_ground_height_band(measurement_cell, coarse_cell, cloud, options);
  const auto density = static_cast<double>(band_points.size()) /
    (coarse_grid.resolution * coarse_grid.resolution);
  if (density < options.refinement_coarse_min_density) {
    return RefineReason::LowCoarseDensity;
  }

  std::vector<double> distances;
  distances.reserve(band_points.size());
  for (const auto point_index : band_points) {
    distances.push_back(point_plane_distance(coarse_cell, cloud.points[point_index]));
  }
  const auto residual_p95 = percentile(std::move(distances), options.refinement_percentile_high);
  if (residual_p95 > options.refinement_coarse_max_plane_distance_p95) {
    return RefineReason::PlaneDistance;
  }
  const auto height_range = percentile_height_range(
    cloud, band_points, options.refinement_percentile_low, options.refinement_percentile_high);
  if (height_range > options.refinement_coarse_max_height_range_p95) {
    return RefineReason::CoarseHeightRange;
  }
  return RefineReason::None;
}

CloudT merge_clouds(const CloudT & localization_cloud, const CloudT & detail_cloud)
{
  CloudT merged;
  merged.points.reserve(localization_cloud.size() + detail_cloud.size());
  merged.points.insert(merged.points.end(), localization_cloud.points.begin(), localization_cloud.points.end());
  merged.points.insert(merged.points.end(), detail_cloud.points.begin(), detail_cloud.points.end());
  merged.width = static_cast<std::uint32_t>(merged.size());
  merged.height = 1;
  merged.is_dense = false;
  return merged;
}

Grid build_refine_target_grid(const Grid & coarse_grid)
{
  auto refine_grid = coarse_grid;
  for (auto & cell : refine_grid.cells) {
    cell.state = cell.refine_target ? CellState::Ground : CellState::Unknown;
    cell.ground_indices.clear();
  }
  return refine_grid;
}

void initialize_candidate_grid_from_coarse(
  Grid & candidate_grid, Grid & traversable_grid, Grid & refine_target_grid, Grid & coarse_grid,
  const Grid & merged_coarse_grid, const CloudT & merged_cloud, const Options & options)
{
  for (std::size_t coarse_y = 0; coarse_y < coarse_grid.height; ++coarse_y) {
    for (std::size_t coarse_x = 0; coarse_x < coarse_grid.width; ++coarse_x) {
      auto & coarse_cell = coarse_grid.cells[flatten_index(coarse_x, coarse_y, coarse_grid.width)];
      const auto & measurement_cell = merged_coarse_grid.cells[flatten_index(coarse_x, coarse_y, merged_coarse_grid.width)];
      if (measurement_cell.point_indices.empty()) {
        continue;
      }
      const auto coarse_is_ground = is_ground_state(coarse_cell.state);
      coarse_cell.refine_reason = classify_refine_coarse_cell(
        coarse_grid, coarse_cell, measurement_cell, merged_cloud, coarse_x, coarse_y, options);
      coarse_cell.refine_target = coarse_cell.refine_reason != RefineReason::None;
      auto & refine_target_cell = refine_target_grid.cells[flatten_index(coarse_x, coarse_y, refine_target_grid.width)];
      refine_target_cell.state = coarse_cell.refine_target ? CellState::Ground : CellState::Unknown;
      refine_target_cell.refine_reason = coarse_cell.refine_reason;

      const auto min_x = coarse_grid.origin_x + static_cast<double>(coarse_x) * coarse_grid.resolution;
      const auto min_y = coarse_grid.origin_y + static_cast<double>(coarse_y) * coarse_grid.resolution;
      const auto max_x = min_x + coarse_grid.resolution;
      const auto max_y = min_y + coarse_grid.resolution;
      std::size_t start_x = 0;
      std::size_t start_y = 0;
      std::size_t end_x = 0;
      std::size_t end_y = 0;
      if (!locate_cell(candidate_grid, min_x, min_y, start_x, start_y)) {
        continue;
      }
      if (!locate_cell(candidate_grid, std::nextafter(max_x, min_x), std::nextafter(max_y, min_y), end_x, end_y)) {
        continue;
      }

      // coarse地面セルを5cm出力グリッドへ展開し、詳細化対象だけ後段で小グリッド評価に回す。
      for (std::size_t y = start_y; y <= end_y; ++y) {
        for (std::size_t x = start_x; x <= end_x; ++x) {
          auto & candidate_cell = candidate_grid.cells[flatten_index(x, y, candidate_grid.width)];
          auto & traversable_cell = traversable_grid.cells[flatten_index(x, y, traversable_grid.width)];
          const auto evaluation_points = coarse_cell.plane_valid ?
            filter_points_in_ground_height_band(candidate_cell, coarse_cell, merged_cloud, options) :
            candidate_cell.point_indices;
          if (coarse_is_ground) {
            candidate_cell.point_indices = evaluation_points;
            candidate_cell.state = CellState::Ground;
          } else {
            candidate_cell.state = CellState::Unknown;
            candidate_cell.ground_indices.clear();
          }
          if (coarse_cell.refine_target) {
            traversable_cell.point_indices = evaluation_points;
            traversable_cell.state = CellState::Candidate;
          } else if (coarse_is_ground) {
            traversable_cell.point_indices = evaluation_points;
            traversable_cell.state = CellState::Ground;
          } else {
            traversable_cell.state = CellState::Unknown;
            traversable_cell.ground_indices.clear();
          }
        }
      }
    }
  }
}

bool fit_refinement_cell_plane(
  const Grid & grid, const CloudT & cloud, const std::size_t cell_x, const std::size_t cell_y,
  const Options & options, Cell & cell)
{
  const auto support = collect_neighbor_points(grid, cell_x, cell_y, options.refinement_plane_fit_radius_cells);
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double rmse = std::numeric_limits<double>::infinity();
  double slope_deg = std::numeric_limits<double>::infinity();
  if (!fit_plane_from_indices(cloud, support, options.refinement_min_plane_points, centroid, normal, rmse, slope_deg)) {
    return false;
  }
  cell.centroid = centroid;
  cell.normal = normal;
  cell.plane_rmse = rmse;
  cell.slope_deg = slope_deg;
  cell.representative_z = mean_z(cloud, cell.point_indices);
  cell.plane_valid = true;
  cell.ground_indices = cell.point_indices;
  return true;
}

void evaluate_refinement_candidates(
  Grid & traversable_grid, const Grid & candidate_grid, const CloudT & merged_cloud, const Options & options,
  RefinementResult & result)
{
  for (std::size_t cell_index = 0; cell_index < traversable_grid.cells.size(); ++cell_index) {
    (void)candidate_grid;
    if (traversable_grid.cells[cell_index].state != CellState::Candidate) {
      continue;
    }
    ++result.refined_cells;
    auto & cell = traversable_grid.cells[cell_index];
    const auto density = static_cast<double>(cell.point_indices.size()) /
      (traversable_grid.resolution * traversable_grid.resolution);
    if (density < options.refinement_min_density) {
      cell.state = CellState::RejectedDensity;
      ++result.density_rejections;
      continue;
    }
    const auto height_range = percentile_height_range(
      merged_cloud, cell.point_indices, options.refinement_percentile_low, options.refinement_percentile_high);
    if (height_range > options.refinement_max_height_range_p95) {
      cell.state = CellState::RejectedHeightRange;
      ++result.height_rejections;
      continue;
    }
    const auto cell_x = cell_index % traversable_grid.width;
    const auto cell_y = cell_index / traversable_grid.width;
    if (!fit_refinement_cell_plane(traversable_grid, merged_cloud, cell_x, cell_y, options, cell) ||
      cell.plane_rmse > options.refinement_max_plane_rmse)
    {
      cell.state = CellState::RejectedRoughness;
      ++result.roughness_rejections;
      continue;
    }
    if (cell.slope_deg > options.refinement_max_slope_deg) {
      cell.state = CellState::RejectedSlope;
      ++result.slope_rejections;
      continue;
    }
    cell.state = CellState::Candidate;
  }
}

bool refinement_connection_ok(
  const Grid & grid, const std::size_t current_x, const std::size_t current_y, const Cell & current,
  const std::size_t neighbor_x, const std::size_t neighbor_y, const Cell & neighbor, const Options & options)
{
  if (!current.plane_valid || !neighbor.plane_valid) {
    return true;
  }
  if (normal_angle_deg(current.normal, neighbor.normal) > options.refinement_max_normal_diff_deg) {
    return false;
  }
  return boundary_height_gap(grid, current_x, current_y, current, neighbor_x, neighbor_y, neighbor) <=
         options.refinement_max_neighbor_height_gap;
}

void grow_refined_ground(
  Grid & traversable_grid, const Grid & coarse_grid, RefinementResult & result, const Options & options)
{
  static constexpr std::array<std::pair<int, int>, 8> kNeighborOffsets{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1},
  }};
  std::deque<std::size_t> queue;

  // 既に白の粗い地面セルに接する小グリッドだけをseedにし、そこから詳細地面連結を広げる。
  for (std::size_t cell_index = 0; cell_index < traversable_grid.cells.size(); ++cell_index) {
    auto & cell = traversable_grid.cells[cell_index];
    if (cell.state != CellState::Candidate) {
      continue;
    }
    const auto cell_x = cell_index % traversable_grid.width;
    const auto cell_y = cell_index / traversable_grid.width;
    bool seed_cell = false;
    const auto [center_x, center_y] = cell_center(traversable_grid, cell_x, cell_y);
    std::size_t coarse_x = 0;
    std::size_t coarse_y = 0;
    if (locate_cell(coarse_grid, center_x, center_y, coarse_x, coarse_y)) {
      seed_cell = coarse_grid.cells[flatten_index(coarse_x, coarse_y, coarse_grid.width)].state == CellState::Seed;
    }
    for (const auto & [offset_x, offset_y] : kNeighborOffsets) {
      const auto neighbor_x = static_cast<std::int64_t>(cell_x) + offset_x;
      const auto neighbor_y = static_cast<std::int64_t>(cell_y) + offset_y;
      if (neighbor_x < 0 || neighbor_y < 0 || neighbor_x >= static_cast<std::int64_t>(traversable_grid.width) ||
        neighbor_y >= static_cast<std::int64_t>(traversable_grid.height))
      {
        continue;
      }
      const auto neighbor_index = flatten_index(
        static_cast<std::size_t>(neighbor_x), static_cast<std::size_t>(neighbor_y), traversable_grid.width);
      if (traversable_grid.cells[neighbor_index].state == CellState::Ground) {
        seed_cell = true;
        break;
      }
    }
    if (seed_cell) {
      cell.state = CellState::Ground;
      queue.push_back(cell_index);
    }
  }

  while (!queue.empty()) {
    const auto current_index = queue.front();
    queue.pop_front();
    const auto current_x = current_index % traversable_grid.width;
    const auto current_y = current_index / traversable_grid.width;
    const auto & current = traversable_grid.cells[current_index];
    for (const auto & [offset_x, offset_y] : kNeighborOffsets) {
      const auto neighbor_x_signed = static_cast<std::int64_t>(current_x) + offset_x;
      const auto neighbor_y_signed = static_cast<std::int64_t>(current_y) + offset_y;
      if (neighbor_x_signed < 0 || neighbor_y_signed < 0 ||
        neighbor_x_signed >= static_cast<std::int64_t>(traversable_grid.width) ||
        neighbor_y_signed >= static_cast<std::int64_t>(traversable_grid.height))
      {
        continue;
      }
      const auto neighbor_x = static_cast<std::size_t>(neighbor_x_signed);
      const auto neighbor_y = static_cast<std::size_t>(neighbor_y_signed);
      const auto neighbor_index = flatten_index(neighbor_x, neighbor_y, traversable_grid.width);
      auto & neighbor = traversable_grid.cells[neighbor_index];
      if (neighbor.state != CellState::Candidate) {
        continue;
      }
      if (!refinement_connection_ok(
          traversable_grid, current_x, current_y, current, neighbor_x, neighbor_y, neighbor, options))
      {
        neighbor.state = CellState::RejectedHeightGap;
        continue;
      }
      neighbor.state = CellState::Ground;
      queue.push_back(neighbor_index);
    }
  }

  for (auto & cell : traversable_grid.cells) {
    if (cell.state == CellState::Candidate) {
      cell.state = CellState::Disconnected;
      ++result.disconnected_cells;
    }
  }
}

RefinementResult run_detail_refinement(
  Grid coarse_grid, const CloudT & localization_cloud, const CloudT & detail_cloud, const Bounds & bounds,
  const Options & options)
{
  RefinementResult result;
  const auto merged_cloud = merge_clouds(localization_cloud, detail_cloud);
  const auto merged_coarse_grid = build_grid(bounds, merged_cloud, options.coarse_resolution);
  result.candidate_grid = build_grid(bounds, merged_cloud, options.refinement_resolution);
  result.traversable_grid = result.candidate_grid;
  result.refine_target_grid = build_refine_target_grid(coarse_grid);

  initialize_candidate_grid_from_coarse(
    result.candidate_grid, result.traversable_grid, result.refine_target_grid, coarse_grid, merged_coarse_grid,
    merged_cloud, options);
  for (const auto & cell : coarse_grid.cells) {
    if (cell.refine_target) {
      ++result.refine_target_coarse_cells;
    }
  }
  if (options.refinement_enabled) {
    evaluate_refinement_candidates(result.traversable_grid, result.candidate_grid, merged_cloud, options, result);
    grow_refined_ground(result.traversable_grid, coarse_grid, result, options);
  }
  result.candidate_cloud = build_cloud_from_ground_cells(result.candidate_grid, merged_cloud);
  result.traversable_cloud = build_cloud_from_ground_cells(result.traversable_grid, merged_cloud);
  return result;
}

void ensure_output_directory(const std::filesystem::path & output_dir)
{
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    throw std::runtime_error("failed to create output directory: " + output_dir.string() + ": " + error.message());
  }
}

void save_ground_cloud(const CloudT & cloud, const std::filesystem::path & output_dir, const char * file_name)
{
  ensure_output_directory(output_dir);
  const auto output_path = output_dir / file_name;
  if (cloud.empty()) {
    std::ofstream file(output_path);
    if (!file) {
      throw std::runtime_error("failed to save empty PCD: " + output_path.string());
    }
    file << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
         << "WIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA ascii\n";
    return;
  }
  if (pcl::io::savePCDFileBinary(output_path.string(), cloud) != 0) {
    throw std::runtime_error("failed to save PCD: " + output_path.string());
  }
}

std::uint8_t pixel_value(const CellState state)
{
  return is_ground_state(state) ? 255 : 0;
}

std::vector<std::uint8_t> build_image_pixels(const Grid & grid)
{
  std::vector<std::uint8_t> pixels(grid.width * grid.height);
  // PGM/PNGは画像上端から書くため、ROS map座標のY正方向と対応するようYを反転する。
  for (std::size_t image_y = 0; image_y < grid.height; ++image_y) {
    const auto grid_y = grid.height - image_y - 1;
    for (std::size_t x = 0; x < grid.width; ++x) {
      pixels[flatten_index(x, image_y, grid.width)] =
        pixel_value(grid.cells[flatten_index(x, grid_y, grid.width)].state);
    }
  }
  return pixels;
}

void write_pgm(const Grid & grid, const std::filesystem::path & output_dir, const char * file_name)
{
  ensure_output_directory(output_dir);
  const auto output_path = output_dir / file_name;
  std::ofstream file(output_path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open PGM: " + output_path.string());
  }
  const auto pixels = build_image_pixels(grid);
  file << "P5\n# CREATOR ai_ship_robot_slam ground_candidate_map_generator\n"
       << grid.width << ' ' << grid.height << "\n255\n";
  file.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  if (!file) {
    throw std::runtime_error("failed to write PGM: " + output_path.string());
  }
}

void write_png(const Grid & grid, const std::filesystem::path & output_dir, const char * file_name)
{
  ensure_output_directory(output_dir);
  const auto output_path = output_dir / file_name;
  const auto pixels = build_image_pixels(grid);
  const cv::Mat image(
    static_cast<int>(grid.height), static_cast<int>(grid.width), CV_8UC1,
    const_cast<std::uint8_t *>(pixels.data()));
  if (!cv::imwrite(output_path.string(), image)) {
    throw std::runtime_error("failed to write PNG: " + output_path.string());
  }
}

cv::Vec3b color_for_traversable_state(const CellState state)
{
  switch (state) {
    case CellState::Seed:
    case CellState::Ground:
      return cv::Vec3b(255, 255, 255);  // white
    case CellState::Candidate:
    case CellState::RefineTarget:
      return cv::Vec3b(255, 255, 0);  // cyan
    case CellState::RejectedDensity:
      return cv::Vec3b(255, 0, 0);  // blue
    case CellState::RejectedHeightRange:
      return cv::Vec3b(0, 0, 255);  // red
    case CellState::RejectedSlope:
      return cv::Vec3b(0, 165, 255);  // orange
    case CellState::RejectedRoughness:
      return cv::Vec3b(255, 0, 255);  // magenta
    case CellState::RejectedNormalDiff:
      return cv::Vec3b(0, 255, 255);  // yellow
    case CellState::RejectedHeightGap:
      return cv::Vec3b(203, 192, 255);  // pink
    case CellState::Disconnected:
      return cv::Vec3b(0, 255, 0);  // green
    case CellState::Unknown:
    default:
      return cv::Vec3b(0, 0, 0);  // black
  }
}

cv::Vec3b color_for_refine_reason(const RefineReason reason)
{
  switch (reason) {
    case RefineReason::DensityContrast:
      return cv::Vec3b(255, 0, 0);  // blue
    case RefineReason::NonGroundCoarse:
      return cv::Vec3b(0, 255, 255);  // yellow
    case RefineReason::NoCoarsePlane:
      return cv::Vec3b(255, 0, 255);  // magenta
    case RefineReason::LowCoarseDensity:
      return cv::Vec3b(255, 128, 0);  // light blue
    case RefineReason::PlaneDistance:
      return cv::Vec3b(0, 165, 255);  // orange
    case RefineReason::CoarseHeightRange:
      return cv::Vec3b(0, 0, 255);  // red
    case RefineReason::None:
    default:
      return cv::Vec3b(0, 0, 0);  // black
  }
}

void write_traversable_reason_png(const Grid & grid, const std::filesystem::path & output_dir, const char * file_name)
{
  ensure_output_directory(output_dir);
  cv::Mat image(static_cast<int>(grid.height), static_cast<int>(grid.width), CV_8UC3);
  for (std::size_t image_y = 0; image_y < grid.height; ++image_y) {
    const auto grid_y = grid.height - image_y - 1;
    for (std::size_t x = 0; x < grid.width; ++x) {
      image.at<cv::Vec3b>(static_cast<int>(image_y), static_cast<int>(x)) =
        color_for_traversable_state(grid.cells[flatten_index(x, grid_y, grid.width)].state);
    }
  }
  const auto output_path = output_dir / file_name;
  if (!cv::imwrite(output_path.string(), image)) {
    throw std::runtime_error("failed to write reason PNG: " + output_path.string());
  }
}

void write_refine_reason_png(const Grid & grid, const std::filesystem::path & output_dir, const char * file_name)
{
  ensure_output_directory(output_dir);
  cv::Mat image(static_cast<int>(grid.height), static_cast<int>(grid.width), CV_8UC3);
  for (std::size_t image_y = 0; image_y < grid.height; ++image_y) {
    const auto grid_y = grid.height - image_y - 1;
    for (std::size_t x = 0; x < grid.width; ++x) {
      image.at<cv::Vec3b>(static_cast<int>(image_y), static_cast<int>(x)) =
        color_for_refine_reason(grid.cells[flatten_index(x, grid_y, grid.width)].refine_reason);
    }
  }
  const auto output_path = output_dir / file_name;
  if (!cv::imwrite(output_path.string(), image)) {
    throw std::runtime_error("failed to write refine reason PNG: " + output_path.string());
  }
}

void write_yaml(
  const Grid & grid, const std::filesystem::path & output_dir, const char * yaml_name, const char * image_name)
{
  ensure_output_directory(output_dir);
  const auto output_path = output_dir / yaml_name;
  std::ofstream file(output_path);
  if (!file) {
    throw std::runtime_error("failed to open YAML: " + output_path.string());
  }
  file << std::fixed << std::setprecision(6);
  file << "image: " << image_name << '\n';
  file << "mode: trinary\n";
  file << "resolution: " << grid.resolution << '\n';
  file << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.000000]\n";
  file << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
}

void save_map_outputs(
  const Grid & grid, const CloudT & cloud, const Options & options, const char * cloud_name, const char * pgm_name,
  const char * png_name, const char * yaml_name)
{
  save_ground_cloud(cloud, options.output_dir, cloud_name);
  write_pgm(grid, options.output_dir, pgm_name);
  if (options.write_debug_png) {
    write_png(grid, options.output_dir, png_name);
  }
  write_yaml(grid, options.output_dir, yaml_name, pgm_name);
}

void save_grid_outputs(
  const Grid & grid, const Options & options, const char * pgm_name, const char * png_name, const char * yaml_name)
{
  write_pgm(grid, options.output_dir, pgm_name);
  if (options.write_debug_png) {
    write_png(grid, options.output_dir, png_name);
  }
  write_yaml(grid, options.output_dir, yaml_name, pgm_name);
}

Stats compute_stats(const Grid & grid, const CloudT & localization_cloud, const CloudT & detail_cloud, const CloudT & output_cloud)
{
  Stats stats;
  stats.localization_points = localization_cloud.size();
  stats.detail_points = detail_cloud.size();
  stats.output_points = output_cloud.size();
  for (const auto & cell : grid.cells) {
    if (!cell.point_indices.empty()) {
      ++stats.observed_cells;
    }
    if (!cell.ground_indices.empty()) {
      ++stats.support_cells;
    }
    if (cell.plane_valid) {
      ++stats.plane_cells;
    }
    if (cell.state == CellState::Seed) {
      ++stats.seed_cells;
    }
    if (cell.state == CellState::Ground) {
      ++stats.ground_cells;
    }
    if (cell.refine_target || cell.state == CellState::RefineTarget) {
      ++stats.refine_target_cells;
    }
    if (is_rejected_state(cell.state)) {
      ++stats.rejected_cells;
    }
  }
  return stats;
}

void print_summary(
  const std::string & label, const Options & options, const Grid & grid, const Stats & stats,
  const char * cloud_name, const char * pgm_name, const char * png_name)
{
  std::cout << "Generated " << label << " ground map\n"
            << "  localization: " << options.localization_path << '\n'
            << "  detail: " << options.detail_path << '\n'
            << "  config: " << (options.config_path.empty() ? std::filesystem::path("<defaults>") : options.config_path) << '\n'
            << "  output_dir: " << options.output_dir << '\n'
            << "  output_cloud: " << (options.output_dir / cloud_name) << '\n'
            << "  output_pgm: " << (options.output_dir / pgm_name) << '\n'
            << "  output_png: "
            << (options.write_debug_png ? (options.output_dir / png_name).string() : std::string("<disabled>")) << '\n'
            << "  resolution: " << grid.resolution << " m\n"
            << "  size: " << grid.width << " x " << grid.height << " cells\n"
            << "  origin: [" << grid.origin_x << ", " << grid.origin_y << "]\n"
            << "  points localization/detail/output: " << stats.localization_points << '/'
            << stats.detail_points << '/' << stats.output_points << '\n'
            << "  cells observed/support/plane/seed/ground/refine/rejected: "
            << stats.observed_cells << '/' << stats.support_cells << '/' << stats.plane_cells << '/'
            << stats.seed_cells << '/' << stats.ground_cells << '/' << stats.refine_target_cells << '/'
            << stats.rejected_cells << '\n';
}

void print_refinement_summary(const RefinementResult & result)
{
  std::cout << "Detail refinement summary\n"
            << "  coarse_refine_targets: " << result.refine_target_coarse_cells << '\n'
            << "  refined_small_cells: " << result.refined_cells << '\n'
            << "  rejections density/height/slope/roughness/disconnected: "
            << result.density_rejections << '/' << result.height_rejections << '/'
            << result.slope_rejections << '/' << result.roughness_rejections << '/'
            << result.disconnected_cells << '\n';
}

}  // namespace
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  try {
    const auto options = ai_ship_robot_slam::parse_options(argc, argv);
    const auto localization_cloud = ai_ship_robot_slam::load_pcd(options.localization_path);
    const auto detail_cloud = ai_ship_robot_slam::load_pcd(options.detail_path);
    const auto bounds = ai_ship_robot_slam::compute_bounds(*localization_cloud);

    auto coarse_result = ai_ship_robot_slam::run_coarse_detection(*localization_cloud, bounds, options);
    ai_ship_robot_slam::save_map_outputs(
      coarse_result.grid, coarse_result.cloud, options, ai_ship_robot_slam::kGroundCoarseCloudName,
      ai_ship_robot_slam::kGroundCoarsePgmName, ai_ship_robot_slam::kGroundCoarsePngName,
      ai_ship_robot_slam::kGroundCoarseYamlName);
    const auto coarse_stats = ai_ship_robot_slam::compute_stats(
      coarse_result.grid, *localization_cloud, *detail_cloud, coarse_result.cloud);
    ai_ship_robot_slam::print_summary(
      "coarse", options, coarse_result.grid, coarse_stats, ai_ship_robot_slam::kGroundCoarseCloudName,
      ai_ship_robot_slam::kGroundCoarsePgmName, ai_ship_robot_slam::kGroundCoarsePngName);

    auto refinement_result = ai_ship_robot_slam::run_detail_refinement(
      std::move(coarse_result.grid), *localization_cloud, *detail_cloud, bounds, options);
    ai_ship_robot_slam::save_map_outputs(
      refinement_result.candidate_grid, refinement_result.candidate_cloud, options,
      ai_ship_robot_slam::kGroundCandidateCloudName, ai_ship_robot_slam::kGroundCandidatePgmName,
      ai_ship_robot_slam::kGroundCandidatePngName, ai_ship_robot_slam::kGroundCandidateYamlName);
    ai_ship_robot_slam::save_map_outputs(
      refinement_result.traversable_grid, refinement_result.traversable_cloud, options,
      ai_ship_robot_slam::kGroundTraversableCloudName, ai_ship_robot_slam::kGroundTraversablePgmName,
      ai_ship_robot_slam::kGroundTraversablePngName, ai_ship_robot_slam::kGroundTraversableYamlName);
    ai_ship_robot_slam::save_grid_outputs(
      refinement_result.refine_target_grid, options, ai_ship_robot_slam::kGroundRefineTargetPgmName,
      ai_ship_robot_slam::kGroundRefineTargetPngName, ai_ship_robot_slam::kGroundRefineTargetYamlName);
    if (options.write_debug_png) {
      ai_ship_robot_slam::write_traversable_reason_png(
        refinement_result.traversable_grid, options.output_dir, ai_ship_robot_slam::kGroundTraversableReasonPngName);
      ai_ship_robot_slam::write_refine_reason_png(
        refinement_result.refine_target_grid, options.output_dir, ai_ship_robot_slam::kGroundRefineReasonPngName);
    }

    const auto traversable_stats = ai_ship_robot_slam::compute_stats(
      refinement_result.traversable_grid, *localization_cloud, *detail_cloud, refinement_result.traversable_cloud);
    ai_ship_robot_slam::print_summary(
      "traversable", options, refinement_result.traversable_grid, traversable_stats,
      ai_ship_robot_slam::kGroundTraversableCloudName, ai_ship_robot_slam::kGroundTraversablePgmName,
      ai_ship_robot_slam::kGroundTraversablePngName);
    ai_ship_robot_slam::print_refinement_summary(refinement_result);
  } catch (const std::exception & error) {
    std::cerr << "ground_candidate_map_generator failed: " << error.what() << '\n';
    ai_ship_robot_slam::print_usage(std::cerr);
    return 1;
  }
  return 0;
}
