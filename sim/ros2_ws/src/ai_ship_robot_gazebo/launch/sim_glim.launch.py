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
    use_mid360_sim_adapter = LaunchConfiguration("use_mid360_sim_adapter")
    sim_lidar_topic = LaunchConfiguration("sim_lidar_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")
    livox_lidar_topic = LaunchConfiguration("livox_lidar_topic")
    livox_imu_topic = LaunchConfiguration("livox_imu_topic")
    use_scan_pattern_line_lookup = LaunchConfiguration("use_scan_pattern_line_lookup")
    input_points_topics = LaunchConfiguration("input_points_topics")
    reference_points_topic = LaunchConfiguration("reference_points_topic")
    fused_points_topic = LaunchConfiguration("fused_points_topic")
    reference_lidar_frame = LaunchConfiguration("reference_lidar_frame")
    fusion_config = LaunchConfiguration("fusion_config")
    glim_config_path = LaunchConfiguration("glim_config_path")
    glim_package = LaunchConfiguration("glim_package")
    glim_executable = LaunchConfiguration("glim_executable")

    # Gazebo側のRVizとodom TFを止め、SLAM backend側の可視化とTFを検証対象にする。
    simulation = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ai_ship_robot_gazebo"), "launch", "simulation.launch.py"])
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
                    "sim_lidar_topic": sim_lidar_topic,
                    "sim_imu_topic": sim_imu_topic,
                    "livox_lidar_topic": livox_lidar_topic,
                    "livox_imu_topic": livox_imu_topic,
                    "use_scan_pattern_line_lookup": use_scan_pattern_line_lookup,
                }.items(),
            )
        ],
    )

    # 本番用glim launchをsimulation側からincludeし、backend側にGazebo依存を持たせない。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ai_ship_robot_slam"), "launch", "glim.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "input_points_topics": input_points_topics,
            "reference_points_topic": reference_points_topic,
            "fused_points_topic": fused_points_topic,
            "reference_lidar_frame": reference_lidar_frame,
            "fusion_config": fusion_config,
            "glim_config_path": glim_config_path,
            "glim_package": glim_package,
            "glim_executable": glim_executable,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="ai_ship_robot"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument("lidar_pattern_file", default_value="lidar_pattern_dual_updown.urdf.xacro"),
            DeclareLaunchArgument("use_mid360_sim_adapter", default_value="true"),
            DeclareLaunchArgument("sim_lidar_topic", default_value="/left_lidar/custom"),
            DeclareLaunchArgument("sim_imu_topic", default_value="/left_lidar/imu"),
            DeclareLaunchArgument("livox_lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("livox_imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("use_scan_pattern_line_lookup", default_value="false"),
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
                    [FindPackageShare("ai_ship_robot_slam"), "config", "glim_sim"]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_gazebo"), "worlds", "lidar_placement.world"]
                ),
            ),
            simulation,
            slam,
        ]
    )
