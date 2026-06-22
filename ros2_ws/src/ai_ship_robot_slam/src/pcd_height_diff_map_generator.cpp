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
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ai_ship_robot_slam
{
namespace
{

enum class CellState : std::uint8_t
{
  Unknown,
  Free,
  Occupied,
};

struct Options
{
  std::filesystem::path input_path;
  std::filesystem::path output_pgm_path;
  double resolution{0.01};
  double height_diff_threshold{0.03};
  std::size_t min_points_per_cell{1};
  std::optional<double> min_z;
  std::optional<double> max_z;
  bool unknown_as_occupied{true};
};

struct Point
{
  double x{};
  double y{};
  double z{};
};

struct Bounds
{
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};

struct PcdLayout
{
  std::vector<std::string> fields;
  std::vector<std::size_t> counts;
  std::size_t x_column{};
  std::size_t y_column{};
  std::size_t z_column{};
  std::size_t total_columns{};
};

struct Cell
{
  std::size_t point_count{};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
  CellState state{CellState::Unknown};
};

struct HeightGrid
{
  std::size_t width{};
  std::size_t height{};
  double origin_x{};
  double origin_y{};
  double resolution{};
  std::vector<Cell> cells;
};

struct MapStats
{
  std::size_t input_points{};
  std::size_t observed_cells{};
  std::size_t free_cells{};
  std::size_t occupied_cells{};
  std::size_t unknown_cells{};
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

double parse_double(const std::string & value, const std::string & name)
{
  std::size_t parsed = 0;
  const auto result = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(name + " must be a finite number: " + value);
  }
  return result;
}

std::size_t parse_size(const std::string & value, const std::string & name)
{
  if (value.empty() || value[0] == '-') {
    throw std::invalid_argument(name + " must be a positive integer: " + value);
  }
  std::size_t parsed = 0;
  const auto result = std::stoull(value, &parsed);
  if (parsed != value.size() || result == 0 || result > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(name + " must be a positive integer: " + value);
  }
  return static_cast<std::size_t>(result);
}

void print_usage(std::ostream & stream)
{
  stream << "Usage: ros2 run ai_ship_robot_slam pcd_height_diff_map_generator ";
  stream << "--input MAP.pcd --output MAP.pgm [OPTIONS]\n\n";
  stream << "Options:\n";
  stream << "  --input PATH                  Input ASCII PCD file.\n";
  stream << "  --output PATH                 Output PGM file. YAML is written next to it.\n";
  stream << "  --resolution M                Grid resolution in meters. Default: 0.01\n";
  stream << "  --height-diff-threshold M     Neighbor height difference threshold. Default: 0.03\n";
  stream << "  --min-points-per-cell N       Minimum points required for observed cells.\n";
  stream << "  --min-z M                     Ignore points below this z value.\n";
  stream << "  --max-z M                     Ignore points above this z value.\n";
  stream << "  --keep-unknown                Write unknown cells as gray instead of occupied.\n";
  stream << "  -h, --help                    Show this help.\n";
}

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

Options parse_options(const int argc, char ** argv)
{
  Options options;
  bool has_input = false;
  bool has_output = false;

  // CLIはROSパラメータではなくオフライン処理用の通常引数として解釈する。
  for (int index = 1; index < argc; ++index) {
    std::string argument = argv[index];
    const auto equals = argument.find('=');
    std::optional<std::string> inline_value;
    if (equals != std::string::npos) {
      inline_value = argument.substr(equals + 1);
      argument = argument.substr(0, equals);
    }

    if (argument == "-h" || argument == "--help") {
      print_usage(std::cout);
      std::exit(0);
    } else if (argument == "--input") {
      options.input_path = take_option_value(argument, inline_value, index, argc, argv);
      has_input = true;
    } else if (argument == "--output") {
      options.output_pgm_path = take_option_value(argument, inline_value, index, argc, argv);
      has_output = true;
    } else if (argument == "--resolution") {
      options.resolution = parse_double(
        take_option_value(argument, inline_value, index, argc, argv), argument);
    } else if (argument == "--height-diff-threshold") {
      options.height_diff_threshold = parse_double(
        take_option_value(argument, inline_value, index, argc, argv), argument);
    } else if (argument == "--min-points-per-cell") {
      options.min_points_per_cell = parse_size(
        take_option_value(argument, inline_value, index, argc, argv), argument);
    } else if (argument == "--min-z") {
      const auto value = take_option_value(argument, inline_value, index, argc, argv);
      options.min_z = parse_double(value, argument);
    } else if (argument == "--max-z") {
      const auto value = take_option_value(argument, inline_value, index, argc, argv);
      options.max_z = parse_double(value, argument);
    } else if (argument == "--keep-unknown") {
      if (inline_value.has_value()) {
        throw std::invalid_argument("--keep-unknown does not take a value");
      }
      options.unknown_as_occupied = false;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }

  // 必須引数と数値範囲を早期に検証し、巨大な点群読み込み前に設定ミスを止める。
  if (!has_input) {
    throw std::invalid_argument("--input is required");
  }
  if (!has_output) {
    throw std::invalid_argument("--output is required");
  }
  if (options.resolution <= 0.0) {
    throw std::invalid_argument("--resolution must be positive");
  }
  if (options.height_diff_threshold <= 0.0) {
    throw std::invalid_argument("--height-diff-threshold must be positive");
  }
  if (options.min_z.has_value() && options.max_z.has_value() && *options.min_z > *options.max_z) {
    throw std::invalid_argument("--min-z must be less than or equal to --max-z");
  }
  if (options.output_pgm_path.extension().empty()) {
    options.output_pgm_path += ".pgm";
  }
  if (options.output_pgm_path.extension() != ".pgm") {
    throw std::invalid_argument("--output must end with .pgm or have no extension");
  }
  return options;
}

std::vector<std::string> read_words_after_key(const std::string & line)
{
  std::istringstream stream(line);
  std::string key;
  stream >> key;

  std::vector<std::string> words;
  std::string word;
  while (stream >> word) {
    words.push_back(word);
  }
  return words;
}

PcdLayout finalize_pcd_layout(PcdLayout layout)
{
  if (layout.fields.empty()) {
    throw std::runtime_error("PCD header does not contain FIELDS");
  }
  if (layout.counts.empty()) {
    layout.counts.assign(layout.fields.size(), 1);
  }
  if (layout.counts.size() != layout.fields.size()) {
    throw std::runtime_error("PCD COUNT size does not match FIELDS size");
  }

  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  std::size_t column = 0;

  // COUNT付きPCDでは1フィールドが複数列を占めるため、x/y/zの実列番号へ展開する。
  for (std::size_t index = 0; index < layout.fields.size(); ++index) {
    if (layout.counts[index] == 0) {
      throw std::runtime_error("PCD COUNT values must be positive");
    }
    if (layout.fields[index] == "x") {
      if (layout.counts[index] != 1) {
        throw std::runtime_error("PCD x field must have COUNT 1");
      }
      layout.x_column = column;
      has_x = true;
    } else if (layout.fields[index] == "y") {
      if (layout.counts[index] != 1) {
        throw std::runtime_error("PCD y field must have COUNT 1");
      }
      layout.y_column = column;
      has_y = true;
    } else if (layout.fields[index] == "z") {
      if (layout.counts[index] != 1) {
        throw std::runtime_error("PCD z field must have COUNT 1");
      }
      layout.z_column = column;
      has_z = true;
    }
    column += layout.counts[index];
  }
  if (!has_x || !has_y || !has_z) {
    throw std::runtime_error("PCD header must contain x, y and z fields");
  }
  layout.total_columns = column;
  return layout;
}

double parse_pcd_scalar(const std::string & token, const std::size_t line_number)
{
  try {
    std::size_t parsed = 0;
    const auto value = std::stod(token, &parsed);
    if (parsed != token.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return value;
  } catch (const std::exception & error) {
    const auto message = "invalid numeric value in PCD data at line " +
      std::to_string(line_number) + ": " + token + " (" + error.what() + ")";
    throw std::runtime_error(message);
  }
}

std::vector<Point> load_ascii_pcd(const std::filesystem::path & path, const Options & options)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open input PCD: " + path.string());
  }

  PcdLayout layout;
  std::string line;
  std::size_t line_number = 0;
  bool data_header_found = false;

  // PCDヘッダはDATA行まで読み、ASCII以外は今回の対象外として明示的に拒否する。
  while (std::getline(file, line)) {
    ++line_number;
    const auto trimmed = trim_copy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }
    std::istringstream stream(trimmed);
    std::string key;
    stream >> key;
    if (key == "FIELDS") {
      layout.fields = read_words_after_key(trimmed);
    } else if (key == "COUNT") {
      layout.counts.clear();
      for (const auto & value : read_words_after_key(trimmed)) {
        layout.counts.push_back(parse_size(value, "PCD COUNT"));
      }
    } else if (key == "DATA") {
      std::string data_type;
      stream >> data_type;
      if (data_type != "ascii") {
        throw std::runtime_error("only ASCII PCD is supported: DATA " + data_type);
      }
      data_header_found = true;
      break;
    }
  }
  if (!data_header_found) {
    throw std::runtime_error("PCD header does not contain DATA ascii");
  }
  layout = finalize_pcd_layout(std::move(layout));

  std::vector<Point> points;
  while (std::getline(file, line)) {
    ++line_number;
    const auto trimmed = trim_copy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    // 行全体を列番号で走査し、x/y/zだけを取り出してZフィルタと有限値チェックを行う。
    std::istringstream stream(trimmed);
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t column = 0; column < layout.total_columns; ++column) {
      std::string token;
      if (!(stream >> token)) {
        const auto message = "PCD data line has fewer columns than expected at line " +
          std::to_string(line_number);
        throw std::runtime_error(message);
      }
      const auto value = parse_pcd_scalar(token, line_number);
      if (column == layout.x_column) {
        x = value;
      } else if (column == layout.y_column) {
        y = value;
      } else if (column == layout.z_column) {
        z = value;
      }
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (options.min_z.has_value() && z < *options.min_z) {
      continue;
    }
    if (options.max_z.has_value() && z > *options.max_z) {
      continue;
    }
    points.push_back(Point{x, y, z});
  }
  if (points.empty()) {
    throw std::runtime_error("PCD does not contain usable finite x/y/z points");
  }
  return points;
}

Bounds compute_bounds(const std::vector<Point> & points)
{
  Bounds bounds;
  for (const auto & point : points) {
    bounds.min_x = std::min(bounds.min_x, point.x);
    bounds.min_y = std::min(bounds.min_y, point.y);
    bounds.max_x = std::max(bounds.max_x, point.x);
    bounds.max_y = std::max(bounds.max_y, point.y);
  }
  return bounds;
}

std::size_t flatten_index(const std::size_t x, const std::size_t y, const std::size_t width)
{
  return y * width + x;
}

HeightGrid build_height_grid(const std::vector<Point> & points, const Options & options)
{
  const auto bounds = compute_bounds(points);
  const auto min_cell_x = static_cast<std::int64_t>(std::floor(bounds.min_x / options.resolution));
  const auto min_cell_y = static_cast<std::int64_t>(std::floor(bounds.min_y / options.resolution));
  const auto max_cell_x = static_cast<std::int64_t>(std::floor(bounds.max_x / options.resolution));
  const auto max_cell_y = static_cast<std::int64_t>(std::floor(bounds.max_y / options.resolution));
  const auto width_signed = max_cell_x - min_cell_x + 1;
  const auto height_signed = max_cell_y - min_cell_y + 1;
  if (width_signed <= 0 || height_signed <= 0) {
    throw std::runtime_error("computed grid size is invalid");
  }

  HeightGrid grid;
  grid.width = static_cast<std::size_t>(width_signed);
  grid.height = static_cast<std::size_t>(height_signed);
  grid.origin_x = static_cast<double>(min_cell_x) * options.resolution;
  grid.origin_y = static_cast<double>(min_cell_y) * options.resolution;
  grid.resolution = options.resolution;
  if (grid.width > std::numeric_limits<std::size_t>::max() / grid.height) {
    throw std::runtime_error("computed grid is too large");
  }
  grid.cells.resize(grid.width * grid.height);

  // 点群をワールド座標に固定した格子へ集約し、各セルの代表高さとして最小Zを保持する。
  for (const auto & point : points) {
    const auto cell_x = static_cast<std::int64_t>(std::floor(point.x / options.resolution));
    const auto cell_y = static_cast<std::int64_t>(std::floor(point.y / options.resolution));
    const auto x = static_cast<std::size_t>(cell_x - min_cell_x);
    const auto y = static_cast<std::size_t>(cell_y - min_cell_y);
    auto & cell = grid.cells[flatten_index(x, y, grid.width)];
    ++cell.point_count;
    cell.min_z = std::min(cell.min_z, point.z);
    cell.max_z = std::max(cell.max_z, point.z);
  }

  return grid;
}

bool is_observed(const Cell & cell, const Options & options)
{
  return cell.point_count >= options.min_points_per_cell;
}

void classify_height_diff_cells(HeightGrid & grid, const Options & options)
{
  std::vector<std::uint8_t> occupied(grid.cells.size(), 0);
  constexpr std::array<std::array<std::int64_t, 2>, 4> directions{
    {{{1, 0}}, {{0, 1}}, {{1, 1}}, {{1, -1}}}};

  // 8近傍を重複なしで比較し、高低差が閾値を超えた境界の両側セルを危険セルにする。
  for (std::size_t y = 0; y < grid.height; ++y) {
    for (std::size_t x = 0; x < grid.width; ++x) {
      const auto index = flatten_index(x, y, grid.width);
      const auto & cell = grid.cells[index];
      if (!is_observed(cell, options)) {
        continue;
      }
      for (const auto & direction : directions) {
        const auto neighbor_x = static_cast<std::int64_t>(x) + direction[0];
        const auto neighbor_y = static_cast<std::int64_t>(y) + direction[1];
        if (neighbor_x < 0 || neighbor_y < 0 ||
          neighbor_x >= static_cast<std::int64_t>(grid.width) ||
          neighbor_y >= static_cast<std::int64_t>(grid.height))
        {
          continue;
        }
        const auto neighbor_index = flatten_index(
          static_cast<std::size_t>(neighbor_x), static_cast<std::size_t>(neighbor_y), grid.width);
        const auto & neighbor = grid.cells[neighbor_index];
        if (!is_observed(neighbor, options)) {
          continue;
        }
        if (std::abs(cell.min_z - neighbor.min_z) > options.height_diff_threshold) {
          occupied[index] = 1;
          occupied[neighbor_index] = 1;
        }
      }
    }
  }

  // 観測済みセルをfree/occupiedへ確定し、点数不足セルはunknownのまま保持する。
  for (std::size_t index = 0; index < grid.cells.size(); ++index) {
    auto & cell = grid.cells[index];
    if (!is_observed(cell, options)) {
      cell.state = CellState::Unknown;
    } else if (occupied[index] != 0) {
      cell.state = CellState::Occupied;
    } else {
      cell.state = CellState::Free;
    }
  }
}

MapStats compute_stats(const HeightGrid & grid)
{
  MapStats stats;
  for (const auto & cell : grid.cells) {
    stats.input_points += cell.point_count;
    if (cell.point_count > 0) {
      ++stats.observed_cells;
    }
    if (cell.state == CellState::Free) {
      ++stats.free_cells;
    } else if (cell.state == CellState::Occupied) {
      ++stats.occupied_cells;
    } else {
      ++stats.unknown_cells;
    }
  }
  return stats;
}

std::uint8_t pixel_value(const CellState state, const Options & options)
{
  if (state == CellState::Free) {
    return 254;
  }
  if (state == CellState::Occupied || options.unknown_as_occupied) {
    return 0;
  }
  return 205;
}

std::filesystem::path yaml_path_for_pgm(const std::filesystem::path & pgm_path)
{
  auto yaml_path = pgm_path;
  yaml_path.replace_extension(".yaml");
  return yaml_path;
}

void ensure_output_directory(const std::filesystem::path & output_path)
{
  const auto parent = output_path.parent_path();
  if (parent.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(parent, error);
  if (error) {
    throw std::runtime_error("failed to create output directory: " + error.message());
  }
}

void write_pgm(const HeightGrid & grid, const Options & options)
{
  ensure_output_directory(options.output_pgm_path);
  std::ofstream file(options.output_pgm_path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open output PGM: " + options.output_pgm_path.string());
  }

  // PGMは画像左上から書くため、ROS mapのY正方向と一致するよう上端行から反転して出力する。
  file << "P5\n"
       << "# CREATOR ai_ship_robot_slam pcd_height_diff_map_generator\n"
       << grid.width << ' ' << grid.height << "\n255\n";
  std::vector<std::uint8_t> pixels(grid.width * grid.height);
  for (std::size_t image_y = 0; image_y < grid.height; ++image_y) {
    const auto grid_y = grid.height - image_y - 1;
    for (std::size_t x = 0; x < grid.width; ++x) {
      const auto grid_index = flatten_index(x, grid_y, grid.width);
      const auto image_index = flatten_index(x, image_y, grid.width);
      pixels[image_index] = pixel_value(grid.cells[grid_index].state, options);
    }
  }
  file.write(
    reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  if (!file) {
    throw std::runtime_error("failed to write output PGM: " + options.output_pgm_path.string());
  }
}

void write_yaml(const HeightGrid & grid, const Options & options)
{
  const auto yaml_path = yaml_path_for_pgm(options.output_pgm_path);
  ensure_output_directory(yaml_path);
  std::ofstream file(yaml_path);
  if (!file) {
    throw std::runtime_error("failed to open output YAML: " + yaml_path.string());
  }

  // map_server互換のYAMLを同じディレクトリのPGM相対パスで出力する。
  file << std::fixed << std::setprecision(6);
  file << "image: " << options.output_pgm_path.filename().string() << '\n';
  file << "mode: trinary\n";
  file << "resolution: " << grid.resolution << '\n';
  file << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.000000]\n";
  file << "negate: 0\n";
  file << "occupied_thresh: 0.65\n";
  file << "free_thresh: 0.196\n";
  if (!file) {
    throw std::runtime_error("failed to write output YAML: " + yaml_path.string());
  }
}

void print_summary(const HeightGrid & grid, const MapStats & stats, const Options & options)
{
  std::cout << "Generated height-diff occupancy grid\n"
            << "  input: " << options.input_path << '\n'
            << "  output_pgm: " << options.output_pgm_path << '\n'
            << "  output_yaml: " << yaml_path_for_pgm(options.output_pgm_path) << '\n'
            << "  resolution: " << grid.resolution << " m\n"
            << "  height_diff_threshold: " << options.height_diff_threshold << " m\n"
            << "  size: " << grid.width << " x " << grid.height << " cells\n"
            << "  origin: [" << grid.origin_x << ", " << grid.origin_y << "]\n"
            << "  points: " << stats.input_points << '\n'
            << "  cells free/occupied/unknown: " << stats.free_cells << '/'
            << stats.occupied_cells << '/' << stats.unknown_cells << '\n';
}

}  // namespace
}  // namespace ai_ship_robot_slam

int main(int argc, char ** argv)
{
  try {
    const auto options = ai_ship_robot_slam::parse_options(argc, argv);
    const auto points = ai_ship_robot_slam::load_ascii_pcd(options.input_path, options);
    auto grid = ai_ship_robot_slam::build_height_grid(points, options);
    ai_ship_robot_slam::classify_height_diff_cells(grid, options);
    const auto stats = ai_ship_robot_slam::compute_stats(grid);
    ai_ship_robot_slam::write_pgm(grid, options);
    ai_ship_robot_slam::write_yaml(grid, options);
    ai_ship_robot_slam::print_summary(grid, stats, options);
  } catch (const std::exception & error) {
    std::cerr << "pcd_height_diff_map_generator failed: " << error.what() << '\n';
    ai_ship_robot_slam::print_usage(std::cerr);
    return 1;
  }
  return 0;
}
