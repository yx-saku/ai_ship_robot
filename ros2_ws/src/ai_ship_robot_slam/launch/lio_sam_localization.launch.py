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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    fusion_config = LaunchConfiguration("fusion_config")
    localization_config = LaunchConfiguration("localization_config")
    rviz_config = LaunchConfiguration("rviz_config")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
    pcd_map_path = LaunchConfiguration("pcd_map_path")
    localization_cloud_topic = LaunchConfiguration("localization_cloud_topic")
    localization_odometry_topic = LaunchConfiguration("localization_odometry_topic")

    # 既存frame名を維持し、Nav2連携時はlidar_initをodom相当として扱える構成にする。
    map_frame = "map"
    lidar_init_frame = "lidar_init"
    lidar_odom_frame = "lidar_odom"
    base_frame = "base_footprint"

    # LIO-SAMは短期odometry生成器として動かし、固定PCD mapの更新や保存は行わない。
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

    # map->lidar_initはlocalizerが動的にpublishするため、static TF側ではbase接続だけを残す。
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
                "publish_map_to_lidar_init": False,
            },
        ],
    )

    # 自己点群除去はmapping時と同じ入力経路を使い、localization固有処理を後段へ限定する。
    livox_custommsg_self_filter = Node(
        package="ai_ship_robot_slam",
        executable="livox_custommsg_self_filter_node",
        name="livox_custommsg_self_filter_node",
        output="screen",
        parameters=[fusion_config, {"use_sim_time": use_sim_time}],
    )

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

    # 完成済みPCDをread-only地図として読み、map->lidar_initの動的TFを単独でpublishする。
    pcd_localization = Node(
        package="ai_ship_robot_slam",
        executable="pcd_localization_node",
        name="pcd_localization_node",
        output="screen",
        parameters=[
            localization_config,
            {
                "use_sim_time": use_sim_time,
                "io.pcd_map_path": pcd_map_path,
                "io.frames.map": map_frame,
                "io.frames.odom": lidar_init_frame,
                "io.frames.lidar": lidar_odom_frame,
                "io.topics.cloud": localization_cloud_topic,
                "io.topics.odometry": localization_odometry_topic,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="lio_sam_localization_rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/lidar1/livox/imu"),
            DeclareLaunchArgument("lio_sam_package", default_value="lio_sam"),
            DeclareLaunchArgument("pcd_map_path", default_value=""),
            DeclareLaunchArgument(
                "localization_cloud_topic",
                default_value="/lio_sam/mapping/cloud_registered",
            ),
            DeclareLaunchArgument(
                "localization_odometry_topic",
                default_value="/lio_sam/mapping/odometry",
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
                "localization_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "pcd_localization.yaml"]
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
            pcd_localization,
            rviz,
        ]
    )
