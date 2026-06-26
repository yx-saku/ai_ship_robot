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

#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ai_ship_robot_slam
{

struct XyziPoint
{
  float x{};
  float y{};
  float z{};
  float intensity{};
};

struct ElevationPoint
{
  double x{};
  double y{};
  double z{};
};

struct ElevationCell
{
  std::int64_t ix{};
  std::int64_t iy{};
  double x{};
  double y{};
  std::size_t count{};
  double z_min{};
  double z_max{};
  double z_mean{};
  double z_m2{};
};

struct ElevationExtractionParams
{
  double cell_size{0.01};
  double cell_z_cluster_gap{0.03};
  double ground_cluster_height_gap{0.05};
  std::size_t ground_cluster_min_cells{10};
};

ElevationPoint to_anchor_yaw_local_map_z(
  double map_x,
  double map_y,
  double map_z,
  double anchor_x,
  double anchor_y,
  double anchor_yaw);

std::vector<ElevationCell> extract_elevation_cells(
  const std::vector<ElevationPoint> & points,
  const ElevationExtractionParams & params);

bool write_binary_xyzi_pcd(
  const std::filesystem::path & path,
  const std::vector<XyziPoint> & points,
  std::string & error_message);

}  // namespace ai_ship_robot_slam
