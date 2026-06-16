import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _format_float(value):
    return f"{value:.16g}"


def _rotation_matrix_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # ROSのstatic_transform_publisherと同じRz(yaw) * Ry(pitch) * Rx(roll)で姿勢を扱う。
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _projected_x_axis_yaw(rotation_matrix):
    x_axis_x = rotation_matrix[0][0]
    x_axis_y = rotation_matrix[1][0]

    # LiDARの+X軸を水平面へ射影し、SLAM初期frameのyawだけを実LiDAR前方へ合わせる。
    if math.hypot(x_axis_x, x_axis_y) <= 1.0e-9:
        return 0.0
    return math.atan2(x_axis_y, x_axis_x)


def _inverse_transform(translation, rotation_matrix):
    inverse_rotation = tuple(tuple(rotation_matrix[row][col] for row in range(3)) for col in range(3))
    inverse_translation = tuple(
        -sum(inverse_rotation[row][col] * translation[col] for col in range(3))
        for row in range(3)
    )
    return inverse_translation


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_fusion = LaunchConfiguration("use_fusion")
    use_adapter = LaunchConfiguration("use_adapter")
    use_map_saver = LaunchConfiguration("use_map_saver")
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
    cloud_map_directory = LaunchConfiguration("cloud_map_directory")

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
    lidar_init_frame = "left_lidar_init"
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

    # URDF既定配置のbase_footprint -> left_lidar_linkから、SLAM初期frameとbase接続TFを一貫して導出する。
    base_to_left_lidar_translation = (0.3925, 0.3275, 0.25)
    base_to_left_lidar_rpy = (0.0, 2.0943951023931953, 0.0)
    base_to_left_lidar_rotation = _rotation_matrix_from_rpy(*base_to_left_lidar_rpy)
    lidar_init_yaw = _projected_x_axis_yaw(base_to_left_lidar_rotation)
    lidar_to_base_translation = _inverse_transform(
        base_to_left_lidar_translation, base_to_left_lidar_rotation
    )

    # left_lidar_initは水平を維持し、並進とLiDAR +X軸の水平投影yawだけを初期姿勢として使う。
    map_to_lidar_init_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_left_lidar_init_static_transform_publisher",
        output="screen",
        arguments=[
            "--x", _format_float(base_to_left_lidar_translation[0]),
            "--y", _format_float(base_to_left_lidar_translation[1]),
            "--z", _format_float(base_to_left_lidar_translation[2]),
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", _format_float(lidar_init_yaw),
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
            "--x", _format_float(lidar_to_base_translation[0]),
            "--y", _format_float(lidar_to_base_translation[1]),
            "--z", _format_float(lidar_to_base_translation[2]),
            "--roll", _format_float(-base_to_left_lidar_rpy[0]),
            "--pitch", _format_float(-base_to_left_lidar_rpy[1]),
            "--yaw", _format_float(-base_to_left_lidar_rpy[2]),
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

    # 登録済み点群をmap frameへ変換して蓄積し、TriggerサービスでPCDとして保存する。
    pcd_map_saver = Node(
        package="ai_ship_robot_slam",
        executable="pcd_map_saver_node",
        name="pcd_map_saver_node",
        output="screen",
        condition=IfCondition(use_map_saver),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cloud_topic": "/lio_sam/mapping/cloud_registered",
                "target_frame": map_frame,
                "output_directory": cloud_map_directory,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_fusion", default_value="false"),
            DeclareLaunchArgument("use_adapter", default_value="false"),
            DeclareLaunchArgument("use_map_saver", default_value="false"),
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
                "cloud_map_directory",
                default_value=os.path.join(
                    os.environ.get("AI_SHIP_ROBOT_WORKSPACE_ROOT", os.getcwd()), "outputs", "cloud_map"
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
            fusion,
            adapter,
            imu_orientation_initializer,
            map_to_lidar_init_tf,
            lidar_odom_to_base_tf,
            imu_preintegration,
            image_projection,
            feature_extraction,
            map_optimization,
            pcd_map_saver,
            rviz,
        ]
    )
