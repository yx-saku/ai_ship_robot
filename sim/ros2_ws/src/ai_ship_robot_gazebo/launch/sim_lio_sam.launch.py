# Copyright 2026 AI Ship Robot Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    gui = LaunchConfiguration("gui")
    lite = LaunchConfiguration("lite")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    lidar_pattern_file = LaunchConfiguration("lidar_pattern_file")
    fusion_config = LaunchConfiguration("fusion_config")
    params_file = LaunchConfiguration("params_file")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
    use_mid360_sim_adapter = LaunchConfiguration("use_mid360_sim_adapter")
    use_scan_pattern_line_lookup = LaunchConfiguration("use_scan_pattern_line_lookup")
    force_zero_offset_time = LaunchConfiguration("force_zero_offset_time")
    input_lidar_reliable = LaunchConfiguration("input_lidar_reliable")
    output_lidar_reliable = LaunchConfiguration("output_lidar_reliable")
    enable_imu_passthrough = LaunchConfiguration("enable_imu_passthrough")

    # Gazebo raw topicをMID360模擬topicへ変換し、live/bagで同じSLAM入力を使う。
    simulation = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ai_ship_robot_gazebo"),
                            "launch",
                            "simulation.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "robot_name": robot_name,
                    "use_rviz": "false",
                    "gui": gui,
                    "lite": lite,
                    "world": world,
                    "lidar_pattern_file": lidar_pattern_file,
                    "publish_odom_tf": "false",
                    "use_mid360_sim_adapter": use_mid360_sim_adapter,
                    "use_scan_pattern_line_lookup": use_scan_pattern_line_lookup,
                    "force_zero_offset_time": force_zero_offset_time,
                    "input_lidar_reliable": input_lidar_reliable,
                    "output_lidar_reliable": output_lidar_reliable,
                    "enable_imu_passthrough": enable_imu_passthrough,
                }.items(),
            )
        ],
    )

    # 本番用LIO-SAM launchをsimulation側からincludeし、SLAM本体にGazebo依存を持たせない。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ai_ship_robot_slam"), "launch", "lio_sam.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "fusion_config": fusion_config,
            "params_file": params_file,
            "imu_topic": imu_topic,
            "lio_sam_package": lio_sam_package,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="ai_ship_robot"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument(
                "lidar_pattern_file",
                default_value="lidar_pattern_dual_front_updown.urdf.xacro",
            ),
            DeclareLaunchArgument("use_mid360_sim_adapter", default_value="true"),
            DeclareLaunchArgument("use_scan_pattern_line_lookup", default_value="false"),
            DeclareLaunchArgument("force_zero_offset_time", default_value="false"),
            DeclareLaunchArgument("input_lidar_reliable", default_value="true"),
            DeclareLaunchArgument("output_lidar_reliable", default_value="true"),
            DeclareLaunchArgument("enable_imu_passthrough", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/lidar1/livox/imu"),
            DeclareLaunchArgument("lio_sam_package", default_value="lio_sam"),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "multi_lidar_fusion.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ai_ship_robot_gazebo"),
                        "worlds",
                        "shipyard_indoor_100x50.world",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "lio_sam_mid360_sim.yaml"]
                ),
            ),
            simulation,
            slam,
        ]
    )
