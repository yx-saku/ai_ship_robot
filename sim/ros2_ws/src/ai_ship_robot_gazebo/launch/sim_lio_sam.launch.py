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
    input_points_topics = LaunchConfiguration("input_points_topics")
    reference_points_topic = LaunchConfiguration("reference_points_topic")
    fused_points_topic = LaunchConfiguration("fused_points_topic")
    lio_custom_topic = LaunchConfiguration("lio_custom_topic")
    reference_lidar_frame = LaunchConfiguration("reference_lidar_frame")
    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_points_topic = LaunchConfiguration("lio_points_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    use_adapter = LaunchConfiguration("use_adapter")
    use_imu_orientation_initializer = LaunchConfiguration("use_imu_orientation_initializer")
    derived_ring_count = LaunchConfiguration("derived_ring_count")
    min_vertical_angle_deg = LaunchConfiguration("min_vertical_angle_deg")
    max_vertical_angle_deg = LaunchConfiguration("max_vertical_angle_deg")
    fusion_timestamp_unit_scale = LaunchConfiguration("fusion_timestamp_unit_scale")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
    use_mid360_sim_adapter = LaunchConfiguration("use_mid360_sim_adapter")
    sim_lidar_topic = LaunchConfiguration("sim_lidar_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")
    livox_lidar_topic = LaunchConfiguration("livox_lidar_topic")
    livox_imu_topic = LaunchConfiguration("livox_imu_topic")
    expected_acceleration_norm = LaunchConfiguration("expected_acceleration_norm")
    acceleration_norm_tolerance = LaunchConfiguration("acceleration_norm_tolerance")
    max_initial_angular_velocity = LaunchConfiguration("max_initial_angular_velocity")
    min_initial_imu_samples = LaunchConfiguration("min_initial_imu_samples")
    min_initial_imu_duration = LaunchConfiguration("min_initial_imu_duration")
    imu_type = LaunchConfiguration("imu_type")
    imu_acceleration_unit = LaunchConfiguration("imu_acceleration_unit")
    imu_acceleration_scale = LaunchConfiguration("imu_acceleration_scale")
    imu_frequency = LaunchConfiguration("imu_frequency")
    imu_debug = LaunchConfiguration("imu_debug")
    wait_for_imu_initialization = LaunchConfiguration("wait_for_imu_initialization")
    use_imu_preintegration_initial_guess = LaunchConfiguration("use_imu_preintegration_initial_guess")
    use_imu_translation_initial_guess = LaunchConfiguration("use_imu_translation_initial_guess")
    use_imu_rotation_initial_guess = LaunchConfiguration("use_imu_rotation_initial_guess")
    deskew_mode = LaunchConfiguration("deskew_mode")
    max_point_offset_time_sec = LaunchConfiguration("max_point_offset_time_sec")

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
                    "use_mid360_sim_adapter": use_mid360_sim_adapter,
                    "sim_lidar_topic": sim_lidar_topic,
                    "sim_imu_topic": sim_imu_topic,
                    "livox_lidar_topic": livox_lidar_topic,
                    "livox_imu_topic": livox_imu_topic,
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
            "input_points_topics": input_points_topics,
            "reference_points_topic": reference_points_topic,
            "fused_points_topic": fused_points_topic,
            "lio_custom_topic": lio_custom_topic,
            "reference_lidar_frame": reference_lidar_frame,
            "raw_imu_topic": raw_imu_topic,
            "imu_topic": imu_topic,
            "lio_points_topic": lio_points_topic,
            "lidar_frame": lidar_frame,
            "base_frame": base_frame,
            "odom_frame": odom_frame,
            "map_frame": map_frame,
            "use_adapter": use_adapter,
            "use_imu_orientation_initializer": use_imu_orientation_initializer,
            "derived_ring_count": derived_ring_count,
            "min_vertical_angle_deg": min_vertical_angle_deg,
            "max_vertical_angle_deg": max_vertical_angle_deg,
            "fusion_timestamp_unit_scale": fusion_timestamp_unit_scale,
            "publish_map_to_odom_tf": publish_map_to_odom_tf,
            "lio_sam_package": lio_sam_package,
            "expected_acceleration_norm": expected_acceleration_norm,
            "acceleration_norm_tolerance": acceleration_norm_tolerance,
            "max_initial_angular_velocity": max_initial_angular_velocity,
            "min_initial_imu_samples": min_initial_imu_samples,
            "min_initial_imu_duration": min_initial_imu_duration,
            "imu_type": imu_type,
            "imu_acceleration_unit": imu_acceleration_unit,
            "imu_acceleration_scale": imu_acceleration_scale,
            "imu_frequency": imu_frequency,
            "imu_debug": imu_debug,
            "wait_for_imu_initialization": wait_for_imu_initialization,
            "use_imu_preintegration_initial_guess": use_imu_preintegration_initial_guess,
            "use_imu_translation_initial_guess": use_imu_translation_initial_guess,
            "use_imu_rotation_initial_guess": use_imu_rotation_initial_guess,
            "deskew_mode": deskew_mode,
            "max_point_offset_time_sec": max_point_offset_time_sec,
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
            DeclareLaunchArgument("input_points_topics", default_value="['/livox/lidar']"),
            DeclareLaunchArgument("reference_points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("fused_points_topic", default_value="/livox/fused_points"),
            DeclareLaunchArgument("lio_custom_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("reference_lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("raw_imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu_oriented"),
            DeclareLaunchArgument("lio_points_topic", default_value="/left_lidar/lio_sam_points"),
            DeclareLaunchArgument("lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_adapter", default_value="false"),
            DeclareLaunchArgument("use_imu_orientation_initializer", default_value="true"),
            DeclareLaunchArgument("derived_ring_count", default_value="4"),
            DeclareLaunchArgument("min_vertical_angle_deg", default_value="-7.22"),
            DeclareLaunchArgument("max_vertical_angle_deg", default_value="55.22"),
            DeclareLaunchArgument("fusion_timestamp_unit_scale", default_value="1.0e-9"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument("lio_sam_package", default_value="lio_sam"),
            DeclareLaunchArgument("expected_acceleration_norm", default_value="1.0"),
            DeclareLaunchArgument("acceleration_norm_tolerance", default_value="0.35"),
            DeclareLaunchArgument("max_initial_angular_velocity", default_value="0.2"),
            DeclareLaunchArgument("min_initial_imu_samples", default_value="50"),
            DeclareLaunchArgument("min_initial_imu_duration", default_value="0.5"),
            DeclareLaunchArgument("imu_type", default_value="six_axis"),
            DeclareLaunchArgument("imu_acceleration_unit", default_value="g"),
            DeclareLaunchArgument("imu_acceleration_scale", default_value="1.0"),
            DeclareLaunchArgument("imu_frequency", default_value="500.0"),
            DeclareLaunchArgument("imu_debug", default_value="false"),
            DeclareLaunchArgument("wait_for_imu_initialization", default_value="true"),
            DeclareLaunchArgument("use_imu_preintegration_initial_guess", default_value="true"),
            DeclareLaunchArgument("use_imu_translation_initial_guess", default_value="false"),
            DeclareLaunchArgument("use_imu_rotation_initial_guess", default_value="true"),
            DeclareLaunchArgument("deskew_mode", default_value="imu_angular"),
            DeclareLaunchArgument("max_point_offset_time_sec", default_value="0.2"),
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
