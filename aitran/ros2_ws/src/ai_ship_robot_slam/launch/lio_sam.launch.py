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
    use_fusion = LaunchConfiguration("use_fusion")
    use_adapter = LaunchConfiguration("use_adapter")
    use_imu_orientation_initializer = LaunchConfiguration("use_imu_orientation_initializer")
    params_file = LaunchConfiguration("params_file")
    fusion_config = LaunchConfiguration("fusion_config")
    rviz_config = LaunchConfiguration("rviz_config")
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
    derived_ring_count = LaunchConfiguration("derived_ring_count")
    min_vertical_angle_deg = LaunchConfiguration("min_vertical_angle_deg")
    max_vertical_angle_deg = LaunchConfiguration("max_vertical_angle_deg")
    fusion_timestamp_unit_scale = LaunchConfiguration("fusion_timestamp_unit_scale")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
    expected_acceleration_norm = LaunchConfiguration("expected_acceleration_norm")
    acceleration_norm_tolerance = LaunchConfiguration("acceleration_norm_tolerance")
    max_initial_angular_velocity = LaunchConfiguration("max_initial_angular_velocity")
    min_initial_imu_samples = LaunchConfiguration("min_initial_imu_samples")
    min_initial_imu_duration = LaunchConfiguration("min_initial_imu_duration")

    # 複数LiDAR fusionは任意機能にし、既定では実機Livox driver topicをLIO-SAMへ直接渡す。
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
                "output_custom_topic": lio_custom_topic,
                "reference_custom_topic": reference_points_topic,
                "reference_lidar_frame": reference_lidar_frame,
                "timestamp_unit_scale": ParameterValue(fusion_timestamp_unit_scale, value_type=float),
            },
        ],
    )

    lio_sam_parameters = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "pointCloudTopic": lio_custom_topic,
            "imuTopic": imu_topic,
            "lidarFrame": lidar_frame,
            "baselinkFrame": base_frame,
            "odometryFrame": odom_frame,
            "mapFrame": map_frame,
        },
    ]

    # Mid-360のPointCloud2差分をLIO-SAMのPointXYZIRT前提へ合わせ、driverやsimulationの出力差を局所化する。
    adapter = Node(
        package="ai_ship_robot_slam",
        executable="mid360_lio_sam_pointcloud_adapter_node",
        name="mid360_lio_sam_pointcloud_adapter_node",
        output="screen",
        condition=IfCondition(use_adapter),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_points_topic": fused_points_topic,
                "output_points_topic": lio_points_topic,
                "output_frame": lidar_frame,
                "derived_ring_count": ParameterValue(derived_ring_count, value_type=int),
                "min_vertical_angle_deg": ParameterValue(min_vertical_angle_deg, value_type=float),
                "max_vertical_angle_deg": ParameterValue(max_vertical_angle_deg, value_type=float),
            }
        ],
    )

    # Mid-360内蔵6軸IMU向けに、静止時加速度から初期roll/pitchだけを推定したIMU topicを作る。
    imu_orientation_initializer = Node(
        package="ai_ship_robot_slam",
        executable="six_axis_imu_initial_orientation_node",
        name="six_axis_imu_initial_orientation_node",
        output="screen",
        condition=IfCondition(use_imu_orientation_initializer),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_imu_topic": raw_imu_topic,
                "output_imu_topic": imu_topic,
                "expected_acceleration_norm": ParameterValue(expected_acceleration_norm, value_type=float),
                "acceleration_norm_tolerance": ParameterValue(acceleration_norm_tolerance, value_type=float),
                "max_initial_angular_velocity_rad_s": ParameterValue(
                    max_initial_angular_velocity, value_type=float
                ),
                "min_initial_samples": ParameterValue(min_initial_imu_samples, value_type=int),
                "min_initial_duration_sec": ParameterValue(min_initial_imu_duration, value_type=float),
            }
        ],
    )

    # 公式LIO-SAM launchと同じmap->odom初期TFを出し、map最適化前のTF木を成立させる。
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_transform_publisher",
        output="screen",
        condition=IfCondition(publish_map_to_odom_tf),
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            map_frame,
            "--child-frame-id",
            odom_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_fusion", default_value="false"),
            DeclareLaunchArgument("use_adapter", default_value="false"),
            DeclareLaunchArgument("use_imu_orientation_initializer", default_value="true"),
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
                default_value=PathJoinSubstitution([FindPackageShare("lio_sam"), "config", "rviz2.rviz"]),
            ),
            fusion,
            adapter,
            imu_orientation_initializer,
            map_to_odom_tf,
            imu_preintegration,
            image_projection,
            feature_extraction,
            map_optimization,
            rviz,
        ]
    )
