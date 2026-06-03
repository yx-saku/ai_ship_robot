# FAST-LIO2 Upstream Record

## Imported Upstream

- Upstream repository: `https://github.com/hku-mars/FAST_LIO.git`
- Upstream component: FAST-LIO / FAST-LIO2
- Upstream commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- Imported date: 2026-06-03
- Imported path: `ros2_ws/src/ai_ship_robot_fast_lio2/upstream/FAST_LIO`
- Imported subset: source, headers, config, launch files, message definitions, RViz config, README, LICENSE, `.gitignore`, `.gitmodules`
- Excluded upstream assets: `.github`, `doc`, `Log`, `PCD`

## Submodules

- Submodule path: `include/ikd-Tree`
- Submodule repository: `https://github.com/hku-mars/ikd-Tree.git`
- Submodule commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`

## License Notes

- Upstream `LICENSE` contains GNU GPL version 2 text.
- Upstream `package.xml` declares `BSD`.
- Treat the imported source conservatively as upstream-licensed code until the license discrepancy is resolved.
- Keep upstream copyright notices, license files, and README content intact when moving or modifying files.

## Current Local Modifications

- No upstream source files have been modified in this import commit.
- Large upstream documentation/assets were intentionally excluded because they are not needed to track ROS 2 porting changes.
- `upstream/COLCON_IGNORE` is added outside the upstream repository root so the ROS 1 upstream package is not built by the ROS 2 workspace scanner.
- This `UPSTREAM.md` records provenance and planned local modifications.

## Planned Local Modifications

- ROS 2 Humble `rclcpp`対応。
- topicを`/livox/lidar`、`/livox/imu`へ統一。
- `sensor_msgs/PointCloud2`入力対応。
- Gazebo点群向けfallback対応。
- 2台LiDAR合成点群入力対応。
- `/slam/odom`、`/slam/path`、`/slam/map_points`出力対応。
- ROS 2 parameter/launch対応。
- ROS 2 map save service対応。
