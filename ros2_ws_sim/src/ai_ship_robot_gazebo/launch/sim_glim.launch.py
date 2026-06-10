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
    glim_config_path = LaunchConfiguration("glim_config_path")
    voxel_leaf_size = LaunchConfiguration("voxel_leaf_size")
    left_points_topic = LaunchConfiguration("left_points_topic")
    right_points_topic = LaunchConfiguration("right_points_topic")
    output_points_topic = LaunchConfiguration("output_points_topic")
    glim_package = LaunchConfiguration("glim_package")
    glim_executable = LaunchConfiguration("glim_executable")

    # Gazebo側のRVizとodom TFを止め、SLAM可視化とGLIM TFを単一の基準にする。
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

    # 本番用GLIM launchをsimulation側からincludeし、SLAM本体はsimulation依存を持たない形にする。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ai_ship_robot_slam"), "launch", "glim.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "glim_config_path": glim_config_path,
            "voxel_leaf_size": voxel_leaf_size,
            "left_points_topic": left_points_topic,
            "right_points_topic": right_points_topic,
            "output_points_topic": output_points_topic,
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
            DeclareLaunchArgument("voxel_leaf_size", default_value="0.03"),
            DeclareLaunchArgument("left_points_topic", default_value="/left_lidar/points"),
            DeclareLaunchArgument("right_points_topic", default_value="/right_lidar/points"),
            DeclareLaunchArgument("output_points_topic", default_value="/slam/points"),
            DeclareLaunchArgument("glim_package", default_value="glim_ros"),
            DeclareLaunchArgument("glim_executable", default_value="glim_rosnode"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_gazebo"), "worlds", "lidar_placement.world"]
                ),
            ),
            DeclareLaunchArgument(
                "glim_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ai_ship_robot_slam"), "config", "glim_sim"]
                ),
            ),
            simulation,
            slam,
        ]
    )
