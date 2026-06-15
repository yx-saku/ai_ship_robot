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
    derived_ring_count = LaunchConfiguration("derived_ring_count")
    min_vertical_angle_deg = LaunchConfiguration("min_vertical_angle_deg")
    max_vertical_angle_deg = LaunchConfiguration("max_vertical_angle_deg")
    fusion_timestamp_unit_scale = LaunchConfiguration("fusion_timestamp_unit_scale")
    lio_sam_package = LaunchConfiguration("lio_sam_package")
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

    # LIO-SAMの推定LiDAR frameを実機URDFのLiDAR frameから分離し、TF親子競合を避ける。
    map_frame = "map"
    lidar_init_frame = "left_lidar-init"
    lidar_odom_frame = "left_lidar_odom"
    base_frame = "base_footprint"

    # YAMLは実機既定値を持ち、launch argumentはsimulationやbag再生時の差分だけを上書きする。
    lio_sam_parameters = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "pointCloudTopic": lio_custom_topic,
            "imuTopic": imu_topic,
            "lidarFrame": lidar_odom_frame,
            "baselinkFrame": base_frame,
            "odometryFrame": lidar_init_frame,
            "mapFrame": map_frame,
            "imuType": imu_type,
            "imuAccelerationUnit": imu_acceleration_unit,
            "imuAccelerationScale": ParameterValue(imu_acceleration_scale, value_type=float),
            "imuFrequency": ParameterValue(imu_frequency, value_type=float),
            "imuDebug": ParameterValue(imu_debug, value_type=bool),
            "waitForImuInitialization": ParameterValue(wait_for_imu_initialization, value_type=bool),
            "initialImuExpectedAccelerationNorm": ParameterValue(
                expected_acceleration_norm, value_type=float
            ),
            "initialImuAccelerationNormTolerance": ParameterValue(
                acceleration_norm_tolerance, value_type=float
            ),
            "initialImuMaxAngularVelocity": ParameterValue(max_initial_angular_velocity, value_type=float),
            "initialImuMinSamples": ParameterValue(min_initial_imu_samples, value_type=int),
            "initialImuMinDuration": ParameterValue(min_initial_imu_duration, value_type=float),
            "useImuPreintegrationInitialGuess": ParameterValue(
                use_imu_preintegration_initial_guess, value_type=bool
            ),
            "useImuTranslationInitialGuess": ParameterValue(
                use_imu_translation_initial_guess, value_type=bool
            ),
            "useImuRotationInitialGuess": ParameterValue(use_imu_rotation_initial_guess, value_type=bool),
            "deskewMode": ParameterValue(deskew_mode, value_type=str),
            "maxPointOffsetTimeSec": ParameterValue(max_point_offset_time_sec, value_type=float),
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
                "output_frame": lidar_odom_frame,
                "derived_ring_count": ParameterValue(derived_ring_count, value_type=int),
                "min_vertical_angle_deg": ParameterValue(min_vertical_angle_deg, value_type=float),
                "max_vertical_angle_deg": ParameterValue(max_vertical_angle_deg, value_type=float),
            }
        ],
    )

    # 最終的なworld/map合わせは外部で調整できるよう、SLAM初期LiDAR frameはmap直下の固定中間frameにする。
    map_to_lidar_init_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_left_lidar_init_static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", map_frame,
            "--child-frame-id", lidar_init_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # LIO-SAMが推定したLiDAR poseからbase_footprintへつなぎ、URDFのロボットlink群へ接続する。
    lidar_odom_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_lidar_odom_to_base_static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.41275635094610965",
            "--y", "-0.3275",
            "--z", "-0.21491496804892735",
            "--roll", "0.0",
            "--pitch", "-2.0943951023931953",
            "--yaw", "0.0",
            "--frame-id", lidar_odom_frame,
            "--child-frame-id", base_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # LIO-SAM本体内の初期化を使わない検証用に、外部nodeでroll/pitchを付与したIMU topicを作る。
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
            DeclareLaunchArgument("use_imu_orientation_initializer", default_value="false"),
            DeclareLaunchArgument("input_points_topics", default_value="['/livox/lidar']"),
            DeclareLaunchArgument("reference_points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("fused_points_topic", default_value="/livox/fused_points"),
            DeclareLaunchArgument("lio_custom_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("reference_lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("raw_imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("lio_points_topic", default_value="/left_lidar/lio_sam_points"),
            DeclareLaunchArgument("derived_ring_count", default_value="4"),
            DeclareLaunchArgument("min_vertical_angle_deg", default_value="-7.22"),
            DeclareLaunchArgument("max_vertical_angle_deg", default_value="55.22"),
            DeclareLaunchArgument("fusion_timestamp_unit_scale", default_value="1.0e-9"),
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
            DeclareLaunchArgument("deskew_mode", default_value="odom_interpolation"),
            DeclareLaunchArgument("max_point_offset_time_sec", default_value="0.2"),
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
            fusion,
            adapter,
            imu_orientation_initializer,
            map_to_lidar_init_tf,
            lidar_odom_to_base_tf,
            imu_preintegration,
            image_projection,
            feature_extraction,
            map_optimization,
            rviz,
        ]
    )
