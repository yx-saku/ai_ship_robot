from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_fusion = LaunchConfiguration("use_fusion")
    fusion_config = LaunchConfiguration("fusion_config")
    glim_config_path = LaunchConfiguration("glim_config_path")
    glim_package = LaunchConfiguration("glim_package")
    glim_executable = LaunchConfiguration("glim_executable")
    input_points_topics = LaunchConfiguration("input_points_topics")
    reference_points_topic = LaunchConfiguration("reference_points_topic")
    fused_points_topic = LaunchConfiguration("fused_points_topic")
    reference_lidar_frame = LaunchConfiguration("reference_lidar_frame")
    rviz_config = LaunchConfiguration("rviz_config")

    # GLIMへ渡す単一点群を作るため、既存のmulti-lidar fusion経路をbackend間で共通利用する。
    fusion = Node(
        package="ai_ship_robot_slam",
        executable="multi_lidar_pointcloud_fusion_node",
        name="multi_lidar_pointcloud_fusion_node",
        output="screen",
        condition=IfCondition(use_fusion),
        parameters=[
            fusion_config,
            {
                "use_sim_time": use_sim_time,
                "input_custom_topics": input_points_topics,
                "output_points_topic": fused_points_topic,
                "reference_custom_topic": reference_points_topic,
                "reference_lidar_frame": reference_lidar_frame,
            },
        ],
    )

    # glim_rosはJSON設定ディレクトリをconfig_pathパラメータで受け取るため、launch側では上書き最小限に留める。
    glim = Node(
        package=glim_package,
        executable=glim_executable,
        name="glim_rosnode",
        output="screen",
        parameters=[
            {
                "config_path": glim_config_path,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="glim_rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_fusion", default_value="true"),
            DeclareLaunchArgument("input_points_topics", default_value="['/livox/lidar']"),
            DeclareLaunchArgument("reference_points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("fused_points_topic", default_value="/livox/fused_points"),
            DeclareLaunchArgument("reference_lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("glim_package", default_value="glim_ros"),
            DeclareLaunchArgument("glim_executable", default_value="glim_rosnode"),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "multi_lidar_fusion.yaml"]
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
            fusion,
            glim,
            rviz,
        ]
    )
