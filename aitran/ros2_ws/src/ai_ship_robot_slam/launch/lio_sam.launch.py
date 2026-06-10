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
    use_adapter = LaunchConfiguration("use_adapter")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    points_topic = LaunchConfiguration("points_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    lio_points_topic = LaunchConfiguration("lio_points_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    derived_ring_count = LaunchConfiguration("derived_ring_count")
    min_vertical_angle_deg = LaunchConfiguration("min_vertical_angle_deg")
    max_vertical_angle_deg = LaunchConfiguration("max_vertical_angle_deg")
    publish_map_to_odom_tf = LaunchConfiguration("publish_map_to_odom_tf")
    lio_sam_package = LaunchConfiguration("lio_sam_package")

    lio_sam_parameters = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "pointCloudTopic": lio_points_topic,
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
                "input_points_topic": points_topic,
                "output_points_topic": lio_points_topic,
                "output_frame": lidar_frame,
                "derived_ring_count": ParameterValue(derived_ring_count, value_type=int),
                "min_vertical_angle_deg": ParameterValue(min_vertical_angle_deg, value_type=float),
                "max_vertical_angle_deg": ParameterValue(max_vertical_angle_deg, value_type=float),
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
            DeclareLaunchArgument("use_adapter", default_value="true"),
            DeclareLaunchArgument("points_topic", default_value="/left_lidar/points"),
            DeclareLaunchArgument("imu_topic", default_value="/left_lidar/imu"),
            DeclareLaunchArgument("lio_points_topic", default_value="/left_lidar/lio_sam_points"),
            DeclareLaunchArgument("lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("derived_ring_count", default_value="4"),
            DeclareLaunchArgument("min_vertical_angle_deg", default_value="-7.22"),
            DeclareLaunchArgument("max_vertical_angle_deg", default_value="55.22"),
            DeclareLaunchArgument("publish_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument("lio_sam_package", default_value="lio_sam"),
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
            adapter,
            map_to_odom_tf,
            imu_preintegration,
            image_projection,
            feature_extraction,
            map_optimization,
            rviz,
        ]
    )
