from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    sim_lidar_topic = LaunchConfiguration("sim_lidar_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")
    livox_lidar_topic = LaunchConfiguration("livox_lidar_topic")
    livox_imu_topic = LaunchConfiguration("livox_imu_topic")
    livox_lidar_frame = LaunchConfiguration("livox_lidar_frame")
    livox_imu_frame = LaunchConfiguration("livox_imu_frame")
    use_scan_pattern_line_lookup = LaunchConfiguration("use_scan_pattern_line_lookup")
    force_zero_offset_time = LaunchConfiguration("force_zero_offset_time")
    input_lidar_reliable = LaunchConfiguration("input_lidar_reliable")
    output_lidar_reliable = LaunchConfiguration("output_lidar_reliable")
    enable_imu_passthrough = LaunchConfiguration("enable_imu_passthrough")
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
            "0.05",
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
                "input_custom_topic": sim_lidar_topic,
                "input_imu_topic": sim_imu_topic,
                "output_custom_topic": livox_lidar_topic,
                "output_imu_topic": livox_imu_topic,
                "output_lidar_frame": livox_lidar_frame,
                "output_imu_frame": livox_imu_frame,
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
            DeclareLaunchArgument("lidar_pattern_file", default_value="lidar_pattern_dual_updown.urdf.xacro"),
            DeclareLaunchArgument("half_lidar_resolution", default_value="false"),
            DeclareLaunchArgument("quarter_lidar_resolution", default_value="false"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_mid360_sim_adapter", default_value="true"),
            DeclareLaunchArgument("sim_lidar_topic", default_value="/left_lidar/custom"),
            DeclareLaunchArgument("sim_imu_topic", default_value="/left_lidar/imu"),
            DeclareLaunchArgument("livox_lidar_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("livox_imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("livox_lidar_frame", default_value="left_lidar_link"),
            DeclareLaunchArgument("livox_imu_frame", default_value="left_lidar_imu_link"),
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
                    [FindPackageShare("ai_ship_robot_gazebo"), "worlds", "lidar_placement.world"]
                ),
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            mid360_sim_adapter,
            rviz,
        ]
    )
