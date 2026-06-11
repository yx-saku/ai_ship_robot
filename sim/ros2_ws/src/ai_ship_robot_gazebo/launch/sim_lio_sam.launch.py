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
    fused_points_topic = LaunchConfiguration("fused_points_topic")
    reference_lidar_frame = LaunchConfiguration("reference_lidar_frame")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_points_topic = LaunchConfiguration("lio_points_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    use_adapter = LaunchConfiguration("use_adapter")
    derived_ring_count = LaunchConfiguration("derived_ring_count")
    min_vertical_angle_deg = LaunchConfiguration("min_vertical_angle_deg")
    max_vertical_angle_deg = LaunchConfiguration("max_vertical_angle_deg")
    fusion_timestamp_unit_scale = LaunchConfiguration("fusion_timestamp_unit_scale")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")
    lio_sam_package = LaunchConfiguration("lio_sam_package")

    # Gazebo側のRVizとodom TFを止め、LIO-SAMが出すmap/odom系TFを検証対象にする。
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
                }.items(),
            )
        ],
    )

    # 本番用LIO-SAM launchをsimulation側からincludeし、SLAM本体にGazebo依存を持たせない。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ai_ship_robot_slam"), "launch", "lio_sam.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "fusion_config": fusion_config,
            "params_file": params_file,
            "fused_points_topic": fused_points_topic,
            "reference_lidar_frame": reference_lidar_frame,
            "imu_topic": imu_topic,
            "lio_points_topic": lio_points_topic,
            "lidar_frame": lidar_frame,
            "base_frame": base_frame,
            "odom_frame": odom_frame,
            "map_frame": map_frame,
            "use_adapter": use_adapter,
            "derived_ring_count": derived_ring_count,
            "min_vertical_angle_deg": min_vertical_angle_deg,
            "max_vertical_angle_deg": max_vertical_angle_deg,
            "fusion_timestamp_unit_scale": fusion_timestamp_unit_scale,
            "publish_map_to_odom_tf": publish_map_to_odom_tf,
            "lio_sam_package": lio_sam_package,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="ai_ship_robot"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument("lidar_pattern_file", default_value="lidar_pattern_dual_updown.urdf.xacro"),
            DeclareLaunchArgument("fused_points_topic", default_value="/left_lidar/fused_points"),
            DeclareLaunchArgument("reference_lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("imu_topic", default_value="/left_lidar/imu"),
            DeclareLaunchArgument("lio_points_topic", default_value="/left_lidar/lio_sam_points"),
            DeclareLaunchArgument("lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_adapter", default_value="true"),
            DeclareLaunchArgument("derived_ring_count", default_value="4"),
            DeclareLaunchArgument("min_vertical_angle_deg", default_value="-7.22"),
            DeclareLaunchArgument("max_vertical_angle_deg", default_value="55.22"),
            DeclareLaunchArgument("fusion_timestamp_unit_scale", default_value="1.0e-9"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
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
                    [FindPackageShare("ai_ship_robot_gazebo"), "worlds", "lidar_placement.world"]
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "lio_sam_mid360.yaml"]
                ),
            ),
            simulation,
            slam,
        ]
    )
