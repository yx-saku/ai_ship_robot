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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_map_saver = LaunchConfiguration("use_map_saver")
    params_file = LaunchConfiguration("params_file")
    fusion_config = LaunchConfiguration("fusion_config")
    rviz_config = LaunchConfiguration("rviz_config")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
    cloud_map_directory = LaunchConfiguration("cloud_map_directory")
    map_cloud_topic = LaunchConfiguration("map_cloud_topic")
    map_odometry_topic = LaunchConfiguration("map_odometry_topic")
    map_path_topic = LaunchConfiguration("map_path_topic")
    map_odometry_sync_tolerance_sec = LaunchConfiguration("map_odometry_sync_tolerance_sec")
    map_cloud_buffer_duration_sec = LaunchConfiguration("map_cloud_buffer_duration_sec")
    hybrid_raw_near_leaf_size = LaunchConfiguration("hybrid_raw_near_leaf_size")
    map_global_voxel_leaf_size = LaunchConfiguration("map_global_voxel_leaf_size")
    map_preview_topic = LaunchConfiguration("map_preview_topic")
    map_preview_publish_period_sec = LaunchConfiguration("map_preview_publish_period_sec")
    map_preview_voxel_leaf_size = LaunchConfiguration("map_preview_voxel_leaf_size")

    # LIO-SAMの推定LiDAR frameを実機URDFのLiDAR frameから分離し、TF親子競合を避ける。
    map_frame = "map"
    lidar_init_frame = "lidar_init"
    lidar_odom_frame = "lidar_odom"
    base_frame = "base_footprint"

    # SLAM挙動はparams_file、multi-LiDAR入力topicはfusion_configで管理する。
    lio_sam_parameters = [
        params_file,
        fusion_config,
        {
            "use_sim_time": use_sim_time,
            "imuTopic": imu_topic,
            "lidarFrame": lidar_odom_frame,
            "baselinkFrame": base_frame,
            "odometryFrame": lidar_init_frame,
            "mapFrame": map_frame,
        },
    ]

    # /tf_staticのbase_footprint -> 基準LiDARから、SLAM初期frameとbase接続TFを一貫して導出する。
    slam_reference_lidar_static_tf = Node(
        package="ai_ship_robot_slam",
        executable="slam_reference_lidar_static_tf_node",
        name="slam_reference_lidar_static_tf_node",
        output="screen",
        parameters=[
            fusion_config,
            {
                "use_sim_time": use_sim_time,
                "base_frame": base_frame,
                "map_frame": map_frame,
                "lidar_init_frame": lidar_init_frame,
                "lidar_odom_frame": lidar_odom_frame,
            }
        ],
    )

    # 自己点群はSLAM投入前に除去し、LIO-SAM本体にはフィルタ後CustomMsgだけを入力する。
    livox_custommsg_self_filter = Node(
        package="ai_ship_robot_slam",
        executable="livox_custommsg_self_filter_node",
        name="livox_custommsg_self_filter_node",
        output="screen",
        parameters=[fusion_config, {"use_sim_time": use_sim_time}],
    )

    # LIO-SAM本体はthird_party underlayのpackageを直接起動し、設定だけをこのproject packageで管理する。
    imu_preintegration = Node(
        package=lio_sam_package,
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        output="screen",
        parameters=lio_sam_parameters,
    )
    image_projection = Node(
        package=lio_sam_package,
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        output="screen",
        parameters=lio_sam_parameters,
    )
    feature_extraction = Node(
        package=lio_sam_package,
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        output="screen",
        parameters=lio_sam_parameters,
    )
    map_optimization = Node(
        package=lio_sam_package,
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        output="screen",
        parameters=lio_sam_parameters,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="lio_sam_rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # local keyframe点群を保持し、保存時に最新path poseでmap frameへ再配置する。
    pcd_map_saver = Node(
        package="ai_ship_robot_slam",
        executable="pcd_map_saver_node",
        name="pcd_map_saver_node",
        output="screen",
        condition=IfCondition(use_map_saver),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cloud_topic": map_cloud_topic,
                "odometry_topic": map_odometry_topic,
                "path_topic": map_path_topic,
                "target_frame": map_frame,
                "output_directory": cloud_map_directory,
                "odometry_sync_tolerance_sec": ParameterValue(
                    map_odometry_sync_tolerance_sec, value_type=float
                ),
                "cloud_buffer_duration_sec": ParameterValue(
                    map_cloud_buffer_duration_sec, value_type=float
                ),
                "submap_voxel_leaf_size": ParameterValue(
                    hybrid_raw_near_leaf_size, value_type=float
                ),
                "global_voxel_leaf_size": ParameterValue(
                    map_global_voxel_leaf_size, value_type=float
                ),
                "preview_enabled": True,
                "preview_topic": map_preview_topic,
                "preview_publish_period_sec": ParameterValue(
                    map_preview_publish_period_sec, value_type=float
                ),
                "preview_voxel_leaf_size": ParameterValue(
                    map_preview_voxel_leaf_size, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_map_saver", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/lidar1/livox/imu"),
            DeclareLaunchArgument("lio_sam_package", default_value="lio_sam"),
            DeclareLaunchArgument(
                "map_cloud_topic", default_value="/lio_sam/mapping/cloud_registered_hybrid"
            ),
            DeclareLaunchArgument("map_odometry_topic", default_value="/lio_sam/mapping/odometry"),
            DeclareLaunchArgument("map_path_topic", default_value="/lio_sam/mapping/path"),
            DeclareLaunchArgument("map_odometry_sync_tolerance_sec", default_value="0.25"),
            DeclareLaunchArgument("map_cloud_buffer_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("hybrid_raw_near_leaf_size", default_value="0.01"),
            DeclareLaunchArgument("map_global_voxel_leaf_size", default_value="0.01"),
            DeclareLaunchArgument(
                "map_preview_topic", default_value="/pcd_map_saver/map_preview"
            ),
            DeclareLaunchArgument("map_preview_publish_period_sec", default_value="2.0"),
            DeclareLaunchArgument("map_preview_voxel_leaf_size", default_value="0.1"),
            DeclareLaunchArgument(
                "cloud_map_directory",
                default_value=os.path.join(
                    os.environ.get("AI_SHIP_ROBOT_WORKSPACE_ROOT", os.getcwd()),
                    "outputs",
                    "cloud_map",
                ),
            ),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "multi_lidar_fusion.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "lio_sam_mid360.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "rviz", "lio_sam.rviz"]
                ),
            ),
            livox_custommsg_self_filter,
            slam_reference_lidar_static_tf,
            imu_preintegration,
            image_projection,
            feature_extraction,
            map_optimization,
            pcd_map_saver,
            rviz,
        ]
    )
