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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cassert>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "ai_ship_robot_slam/map_saver_utils.hpp"

namespace
{
void test_lowest_z_cluster_is_retained()
{
  std::vector<ai_ship_robot_slam::ElevationPoint> points;
  for (int ix = 0; ix < 3; ++ix) {
    for (int iy = 0; iy < 3; ++iy) {
      points.push_back({static_cast<double>(ix) + 0.1, static_cast<double>(iy) + 0.1, 0.00});
      points.push_back({static_cast<double>(ix) + 0.1, static_cast<double>(iy) + 0.1, 0.02});
      points.push_back({static_cast<double>(ix) + 0.1, static_cast<double>(iy) + 0.1, 0.20});
    }
  }

  ai_ship_robot_slam::ElevationExtractionParams params;
  params.cell_size = 1.0;
  params.cell_z_cluster_gap = 0.03;
  params.ground_cluster_height_gap = 0.05;
  params.ground_cluster_min_cells = 1;
  const auto cells = ai_ship_robot_slam::extract_elevation_cells(points, params);

  // 各セルでは0.20mの上側クラスタが除外され、低い2点だけが統計に入る。
  assert(cells.size() == 9U);
  for (const auto & cell : cells) {
    assert(cell.count == 2U);
    assert(std::abs(cell.z_min - 0.00) < 1.0e-9);
    assert(std::abs(cell.z_max - 0.02) < 1.0e-9);
    assert(std::abs(cell.z_mean - 0.01) < 1.0e-9);
  }
}

void test_largest_connected_component_is_selected()
{
  std::vector<ai_ship_robot_slam::ElevationPoint> points;
  for (int ix = 0; ix < 3; ++ix) {
    for (int iy = 0; iy < 3; ++iy) {
      points.push_back({static_cast<double>(ix) + 0.1, static_cast<double>(iy) + 0.1, 0.0});
    }
  }
  points.push_back({10.1, 10.1, 0.0});
  points.push_back({11.1, 10.1, 0.0});
  points.push_back({12.1, 10.1, 0.0});

  ai_ship_robot_slam::ElevationExtractionParams params;
  params.cell_size = 1.0;
  params.cell_z_cluster_gap = 0.03;
  params.ground_cluster_height_gap = 0.05;
  params.ground_cluster_min_cells = 1;
  const auto cells = ai_ship_robot_slam::extract_elevation_cells(points, params);

  // 8近傍連結後は、3セルの小成分ではなく9セルの最大成分だけを残す。
  assert(cells.size() == 9U);
  for (const auto & cell : cells) {
    assert(cell.ix >= 0 && cell.ix <= 2);
    assert(cell.iy >= 0 && cell.iy <= 2);
  }
}

void test_min_cell_filter_removes_small_components()
{
  std::vector<ai_ship_robot_slam::ElevationPoint> points{{0.1, 0.1, 0.0}, {1.1, 0.1, 0.0}};
  ai_ship_robot_slam::ElevationExtractionParams params;
  params.cell_size = 1.0;
  params.ground_cluster_min_cells = 3;
  const auto cells = ai_ship_robot_slam::extract_elevation_cells(points, params);

  // 最小セル数未満の孤立成分だけなら、elevation submapには何も残さない。
  assert(cells.empty());
}

void test_anchor_yaw_local_projection_keeps_map_z()
{
  constexpr double half_pi = 1.57079632679489661923;
  const auto point = ai_ship_robot_slam::to_anchor_yaw_local_map_z(
    1.0, 2.0, 3.0, 1.0, 1.0, half_pi);

  // yaw 90度のanchorではmap上の+Yがlocal +Xへ移り、zはmap frame値のまま残る。
  assert(std::abs(point.x - 1.0) < 1.0e-9);
  assert(std::abs(point.y - 0.0) < 1.0e-9);
  assert(std::abs(point.z - 3.0) < 1.0e-9);
}

void test_binary_pcd_is_readable_by_pcl()
{
  const auto path = std::filesystem::temp_directory_path() /
    "ai_ship_robot_map_saver_utils_test.pcd";
  const std::vector<ai_ship_robot_slam::XyziPoint> points{
    {1.0F, 2.0F, 3.0F, 4.0F}, {5.0F, 6.0F, 7.0F, 8.0F}};
  std::string error_message;
  assert(ai_ship_robot_slam::write_binary_xyzi_pcd(path, points, error_message));

  pcl::PointCloud<pcl::PointXYZI> loaded;
  assert(pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), loaded) == 0);
  assert(loaded.size() == points.size());
  assert(std::abs(loaded.points[0].x - 1.0F) < 1.0e-6F);
  assert(std::abs(loaded.points[1].intensity - 8.0F) < 1.0e-6F);
  std::filesystem::remove(path);
}
}  // namespace

int main()
{
  test_lowest_z_cluster_is_retained();
  test_largest_connected_component_is_selected();
  test_min_cell_filter_removes_small_components();
  test_anchor_yaw_local_projection_keeps_map_z();
  test_binary_pcd_is_readable_by_pcl();
  return 0;
}
