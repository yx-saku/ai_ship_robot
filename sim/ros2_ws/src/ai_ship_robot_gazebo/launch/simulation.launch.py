from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    lite = LaunchConfiguration("lite")
    lidar_pattern_file = LaunchConfiguration("lidar_pattern_file")
    half_lidar_resolution = LaunchConfiguration("half_lidar_resolution")
    quarter_lidar_resolution = LaunchConfiguration("quarter_lidar_resolution")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_mid360_sim_adapter = LaunchConfiguration("use_mid360_sim_adapter")
    cmd_vel_input_topic = LaunchConfiguration("cmd_vel_input_topic")
    cmd_vel_internal_topic = LaunchConfiguration("cmd_vel_internal_topic")
    use_scan_pattern_line_lookup = LaunchConfiguration("use_scan_pattern_line_lookup")
    force_zero_offset_time = LaunchConfiguration("force_zero_offset_time")
    input_lidar_reliable = LaunchConfiguration("input_lidar_reliable")
    output_lidar_reliable = LaunchConfiguration("output_lidar_reliable")
    enable_imu_passthrough = LaunchConfiguration("enable_imu_passthrough")
    max_linear_acceleration_mps2 = LaunchConfiguration("max_linear_acceleration_mps2")
    max_angular_acceleration_radps2 = LaunchConfiguration("max_angular_acceleration_radps2")
    gazebo_params_file = PathJoinSubstitution(
        [FindPackageShare("ai_ship_robot_gazebo"), "config", "gazebo_ros.yaml"]
    )
    effective_gui = PythonExpression(["'false' if '", lite, "' == 'true' else '", gui, "'"])
    effective_half_lidar_resolution = PythonExpression(
        [
            "'true' if '",
            half_lidar_resolution,
            "' == 'true' else 'false'",
        ]
    )
    effective_quarter_lidar_resolution = PythonExpression(
        [
            "'true' if '",
            quarter_lidar_resolution,
            "' == 'true' or ('",
            lite,
            "' == 'true' and '",
            half_lidar_resolution,
            "' != 'true' and '",
            quarter_lidar_resolution,
            "' != 'true') else 'false'",
        ]
    )

    # Gazebo Classicでも同じxacro引数でLiDAR負荷を切り替えられるようにする。
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_description"), "urdf", "ai_ship_robot.urdf.xacro"]
                ),
                " use_sim:=true",
                " lidar_pattern_file:=",
                lidar_pattern_file,
                " half_lidar_resolution:=",
                effective_half_lidar_resolution,
                " quarter_lidar_resolution:=",
                effective_quarter_lidar_resolution,
                " publish_odom_tf:=",
                publish_odom_tf,
            ]
        ),
        value_type=str,
    )

    # Gazebo Classicの標準launchを使い、worldとGUI設定だけを明示的に渡す。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "gui": effective_gui,
            "verbose": verbose,
            "params_file": gazebo_params_file,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    # 生成済みrobot_descriptionをClassic側へ直接spawnする。
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.22",
        ],
    )

    # 外部 /cmd_vel を内部制限topicへ中継し、linear.y の無効化を一元化する。
    cmd_vel_slope_adapter = Node(
        package="ai_ship_robot_gazebo",
        executable="cmd_vel_slope_adapter",
        name="cmd_vel_slope_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": cmd_vel_input_topic,
                "output_topic": cmd_vel_internal_topic,
                "max_linear_acceleration_mps2": max_linear_acceleration_mps2,
                "max_angular_acceleration_radps2": max_angular_acceleration_radps2,
            }
        ],
    )

    # Gazebo 内の controller_manager に対して broadcaster と wheel velocity controller を順次spawnする。
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    slope_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["slope_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 内部 cmd_vel を wheel 速度へ配分し、wheel state から /odom と必要TFを生成する。
    slope_drive_bridge = Node(
        package="ai_ship_robot_gazebo",
        executable="slope_drive_bridge",
        name="slope_drive_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": cmd_vel_internal_topic,
                "controller_command_topic": "/slope_drive_controller/commands",
                "odom_topic": "/odom",
                "odometry_frame": "odom",
                "robot_base_frame": "base_footprint",
                "publish_odom_tf": publish_odom_tf,
                "wheel_radius": 0.04,
                "wheelbase": 0.52,
                "track_width": 0.54,
                "left_wheels": ["left_front_wheel_joint", "left_rear_wheel_joint"],
                "right_wheels": ["right_front_wheel_joint", "right_rear_wheel_joint"],
            }
        ],
    )

    # Gazebo pluginのraw topicをSLAM用の理想LiDARとして整形し、点群内時刻差を持たない入力にする。
    mid360_sim_adapter = Node(
        package="ai_ship_robot_gazebo",
        executable="mid360_sim_adapter",
        name="mid360_sim_adapter",
        output="screen",
        condition=IfCondition(use_mid360_sim_adapter),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_custom_topics": [
                    "/lidar1/custom",
                    "/lidar2/custom",
                    "/lidar3/custom",
                    "/lidar4/custom",
                ],
                "input_imu_topics": [
                    "/lidar1/imu",
                    "/lidar2/imu",
                    "/lidar3/imu",
                    "/lidar4/imu",
                ],
                "output_custom_topics": [
                    "/lidar1/livox/lidar",
                    "/lidar2/livox/lidar",
                    "/lidar3/livox/lidar",
                    "/lidar4/livox/lidar",
                ],
                "output_imu_topics": [
                    "/lidar1/livox/imu",
                    "/lidar2/livox/imu",
                    "/lidar3/livox/imu",
                    "/lidar4/livox/imu",
                ],
                "output_lidar_frames": ["lidar1_link", "lidar2_link", "lidar3_link", "lidar4_link"],
                "output_imu_frames": [
                    "lidar1_imu_link",
                    "lidar2_imu_link",
                    "lidar3_imu_link",
                    "lidar4_imu_link",
                ],
                "use_scan_pattern_line_lookup": use_scan_pattern_line_lookup,
                "force_zero_offset_time": force_zero_offset_time,
                "input_lidar_reliable": input_lidar_reliable,
                "output_lidar_reliable": output_lidar_reliable,
                "enable_imu_passthrough": enable_imu_passthrough,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="ai_ship_robot"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument("lite", default_value="false"),
            DeclareLaunchArgument("lidar_pattern_file", default_value="lidar_pattern_dual_opposit.urdf.xacro"),
            DeclareLaunchArgument("half_lidar_resolution", default_value="false"),
            DeclareLaunchArgument("quarter_lidar_resolution", default_value="false"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_mid360_sim_adapter", default_value="true"),
            DeclareLaunchArgument("cmd_vel_input_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_internal_topic", default_value="/cmd_vel_slope_limited"),
            DeclareLaunchArgument("max_linear_acceleration_mps2", default_value="0.3"),
            DeclareLaunchArgument("max_angular_acceleration_radps2", default_value="0.9"),
            DeclareLaunchArgument("use_scan_pattern_line_lookup", default_value="false"),
            DeclareLaunchArgument("force_zero_offset_time", default_value="true"),
            DeclareLaunchArgument("input_lidar_reliable", default_value="true"),
            DeclareLaunchArgument("output_lidar_reliable", default_value="true"),
            DeclareLaunchArgument("enable_imu_passthrough", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_gazebo"), "config", "mid360_points.rviz"]
                ),
            ),
             DeclareLaunchArgument(
                 "world",
                 default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ai_ship_robot_gazebo"),
                        "worlds",
                        "shipyard_indoor_100x50_light.world",
                    ]
                 ),
             ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            cmd_vel_slope_adapter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[slope_drive_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=slope_drive_controller_spawner,
                    on_exit=[slope_drive_bridge],
                )
            ),
            mid360_sim_adapter,
            rviz,
        ]
    )
