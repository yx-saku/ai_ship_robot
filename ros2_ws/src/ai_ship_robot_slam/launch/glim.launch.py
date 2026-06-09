from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    fusion_config = LaunchConfiguration("fusion_config")
    glim_config_path = LaunchConfiguration("glim_config_path")
    glim_package = LaunchConfiguration("glim_package")
    glim_executable = LaunchConfiguration("glim_executable")
    left_points_topic = LaunchConfiguration("left_points_topic")
    right_points_topic = LaunchConfiguration("right_points_topic")
    output_points_topic = LaunchConfiguration("output_points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    target_frame = LaunchConfiguration("target_frame")
    voxel_leaf_size = LaunchConfiguration("voxel_leaf_size")

    # 2台LiDAR入力をSLAM用の単一PointCloud2へ正規化し、実機でもtopic差し替えだけで再利用できるようにする。
    fusion_node = Node(
        package="ai_ship_robot_slam",
        executable="dual_lidar_pointcloud_fusion_node",
        name="dual_lidar_pointcloud_fusion_node",
        output="screen",
        parameters=[
            fusion_config,
            {
                "use_sim_time": use_sim_time,
                "left_points_topic": left_points_topic,
                "right_points_topic": right_points_topic,
                "output_points_topic": output_points_topic,
                "target_frame": target_frame,
                "voxel_leaf_size": voxel_leaf_size,
            },
        ],
    )

    # glim_rosはJSON設定ディレクトリをconfig_pathパラメータで受け取るため、ROS params YAMLとして渡さない。
    glim_node = Node(
        package=glim_package,
        executable=glim_executable,
        name="glim",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "config_path": glim_config_path,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="slam_rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("left_points_topic", default_value="/left_lidar/points"),
            DeclareLaunchArgument("right_points_topic", default_value="/right_lidar/points"),
            DeclareLaunchArgument("output_points_topic", default_value="/slam/points"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("voxel_leaf_size", default_value="0.03"),
            DeclareLaunchArgument("glim_package", default_value="glim_ros"),
            DeclareLaunchArgument("glim_executable", default_value="glim_rosnode"),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "dual_lidar_fusion.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "glim_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "glim_real"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "rviz", "glim_slam.rviz"]
                ),
            ),
            fusion_node,
            glim_node,
            rviz,
        ]
    )
