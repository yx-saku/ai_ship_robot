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
#include <fstream>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "ai_ship_robot_slam/map_saver_utils.hpp"

namespace ai_ship_robot_slam
{
namespace
{
struct GridKey
{
  std::int64_t ix{};
  std::int64_t iy{};

  bool operator==(const GridKey & other) const
  {
    return ix == other.ix && iy == other.iy;
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & key) const
  {
    const auto hx = std::hash<std::int64_t>{}(key.ix);
    const auto hy = std::hash<std::int64_t>{}(key.iy);
    return hx ^ (hy << 1U);
  }
};

struct CellCandidate
{
  std::vector<double> retained_z;
  double representative_z{};
};

struct RunningStats
{
  std::size_t count{};
  double z_min{std::numeric_limits<double>::infinity()};
  double z_max{-std::numeric_limits<double>::infinity()};
  double z_mean{};
  double z_m2{};

  void add(const double z)
  {
    ++count;
    z_min = std::min(z_min, z);
    z_max = std::max(z_max, z);
    const double delta = z - z_mean;
    z_mean += delta / static_cast<double>(count);
    const double delta_after = z - z_mean;
    z_m2 += delta * delta_after;
  }
};

struct ComponentSummary
{
  std::vector<GridKey> cells;
  std::size_t point_count{};
  double z_sum{};

  double mean_z() const
  {
    if (point_count == 0U) {
      return std::numeric_limits<double>::infinity();
    }
    return z_sum / static_cast<double>(point_count);
  }
};

std::vector<double> lowest_z_cluster(std::vector<double> values, const double cluster_gap)
{
  if (values.empty()) {
    return {};
  }

  // z昇順の最初の連続クラスタだけを残し、上側障害物点が地面統計へ混ざるのを防ぐ。
  std::sort(values.begin(), values.end());
  std::size_t end_index = 1U;
  while (end_index < values.size() && values[end_index] - values[end_index - 1U] <= cluster_gap) {
    ++end_index;
  }
  values.resize(end_index);
  return values;
}

bool is_better_component(
  const ComponentSummary & candidate,
  const ComponentSummary & current_best,
  const bool has_current_best)
{
  if (!has_current_best) {
    return true;
  }
  if (candidate.cells.size() != current_best.cells.size()) {
    return candidate.cells.size() > current_best.cells.size();
  }
  if (candidate.point_count != current_best.point_count) {
    return candidate.point_count > current_best.point_count;
  }
  return candidate.mean_z() < current_best.mean_z();
}
}  // namespace

ElevationPoint to_anchor_yaw_local_map_z(
  const double map_x,
  const double map_y,
  const double map_z,
  const double anchor_x,
  const double anchor_y,
  const double anchor_yaw)
{
  const double dx = map_x - anchor_x;
  const double dy = map_y - anchor_y;
  const double cos_yaw = std::cos(anchor_yaw);
  const double sin_yaw = std::sin(anchor_yaw);

  // XYだけをanchor yaw-localへ回し、zはmap/path frameの絶対高さとして保持する。
  return ElevationPoint{
    cos_yaw * dx + sin_yaw * dy,
    -sin_yaw * dx + cos_yaw * dy,
    map_z};
}

std::vector<ElevationCell> extract_elevation_cells(
  const std::vector<ElevationPoint> & points,
  const ElevationExtractionParams & params)
{
  const double cell_size = params.cell_size > 0.0 ? params.cell_size : 0.01;
  const double cluster_gap = std::max(0.0, params.cell_z_cluster_gap);
  const double component_gap = std::max(0.0, params.ground_cluster_height_gap);
  const std::size_t min_cells = std::max<std::size_t>(1U, params.ground_cluster_min_cells);
  std::unordered_map<GridKey, std::vector<double>, GridKeyHash> raw_cells;
  raw_cells.reserve(points.size());

  // raw点をanchor yaw-local gridへ集約し、セル内zクラスタリングの入力だけを保持する。
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const GridKey key{
      static_cast<std::int64_t>(std::floor(point.x / cell_size)),
      static_cast<std::int64_t>(std::floor(point.y / cell_size))};
    raw_cells[key].push_back(point.z);
  }

  std::unordered_map<GridKey, CellCandidate, GridKeyHash> candidates;
  candidates.reserve(raw_cells.size());
  for (auto & [key, z_values] : raw_cells) {
    auto retained_z = lowest_z_cluster(std::move(z_values), cluster_gap);
    if (retained_z.empty()) {
      continue;
    }
    const double z_sum = std::accumulate(retained_z.begin(), retained_z.end(), 0.0);
    const double representative_z = z_sum / static_cast<double>(retained_z.size());
    candidates.emplace(
      key, CellCandidate{std::move(retained_z), representative_z});
  }

  std::unordered_set<GridKey, GridKeyHash> visited;
  ComponentSummary best_component;
  bool has_best_component = false;
  const std::array<GridKey, 8U> neighbor_offsets{{
    GridKey{-1, -1}, GridKey{-1, 0}, GridKey{-1, 1}, GridKey{0, -1},
    GridKey{0, 1}, GridKey{1, -1}, GridKey{1, 0}, GridKey{1, 1}}};

  // 8近傍で代表高さが近いセルを連結し、最大の地面候補成分だけをCSV対象にする。
  for (const auto & [start_key, start_cell] : candidates) {
    (void)start_cell;
    if (visited.find(start_key) != visited.end()) {
      continue;
    }
    ComponentSummary component;
    std::queue<GridKey> queue;
    queue.push(start_key);
    visited.insert(start_key);

    while (!queue.empty()) {
      const auto key = queue.front();
      queue.pop();
      const auto cell_iter = candidates.find(key);
      if (cell_iter == candidates.end()) {
        continue;
      }
      component.cells.push_back(key);
      component.point_count += cell_iter->second.retained_z.size();
      component.z_sum += std::accumulate(
        cell_iter->second.retained_z.begin(), cell_iter->second.retained_z.end(), 0.0);

      for (const auto & offset : neighbor_offsets) {
        const GridKey neighbor{key.ix + offset.ix, key.iy + offset.iy};
        if (visited.find(neighbor) != visited.end()) {
          continue;
        }
        const auto neighbor_iter = candidates.find(neighbor);
        if (neighbor_iter == candidates.end()) {
          continue;
        }
        if (std::abs(
            neighbor_iter->second.representative_z - cell_iter->second.representative_z) <=
          component_gap)
        {
          visited.insert(neighbor);
          queue.push(neighbor);
        }
      }
    }

    if (component.cells.size() >= min_cells &&
      is_better_component(component, best_component, has_best_component))
    {
      best_component = std::move(component);
      has_best_component = true;
    }
  }

  if (!has_best_component) {
    return {};
  }

  std::vector<ElevationCell> cells;
  cells.reserve(best_component.cells.size());
  for (const auto & key : best_component.cells) {
    const auto candidate_iter = candidates.find(key);
    if (candidate_iter == candidates.end()) {
      continue;
    }
    RunningStats stats;
    for (const auto z : candidate_iter->second.retained_z) {
      stats.add(z);
    }
    cells.push_back(
      ElevationCell{
        key.ix,
        key.iy,
        (static_cast<double>(key.ix) + 0.5) * cell_size,
        (static_cast<double>(key.iy) + 0.5) * cell_size,
        stats.count,
        stats.z_min,
        stats.z_max,
        stats.z_mean,
        stats.z_m2});
  }
  std::sort(
    cells.begin(), cells.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.ix != rhs.ix) {
        return lhs.ix < rhs.ix;
      }
      return lhs.iy < rhs.iy;
    });
  return cells;
}

bool write_binary_xyzi_pcd(
  const std::filesystem::path & path,
  const std::vector<XyziPoint> & points,
  std::string & error_message)
{
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    error_message = "failed to open binary PCD file: " + path.string();
    return false;
  }

  // PCL互換のPCD v0.7 binaryヘッダを出力し、XYZIをfloat32の連続データとして保存する。
  file << "# .PCD v0.7 - Point Cloud Data file format\n";
  file << "VERSION 0.7\n";
  file << "FIELDS x y z intensity\n";
  file << "SIZE 4 4 4 4\n";
  file << "TYPE F F F F\n";
  file << "COUNT 1 1 1 1\n";
  file << "WIDTH " << points.size() << "\n";
  file << "HEIGHT 1\n";
  file << "VIEWPOINT 0 0 0 1 0 0 0\n";
  file << "POINTS " << points.size() << "\n";
  file << "DATA binary\n";
  for (const auto & point : points) {
    file.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
    file.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
    file.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
    file.write(reinterpret_cast<const char *>(&point.intensity), sizeof(float));
  }

  if (!file) {
    error_message = "failed to write binary PCD file: " + path.string();
    return false;
  }
  return true;
}

}  // namespace ai_ship_robot_slam
