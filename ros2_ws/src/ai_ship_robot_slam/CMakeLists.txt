cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mid360_lio_sam_pointcloud_adapter_node src/mid360_lio_sam_pointcloud_adapter_node.cpp)
target_compile_features(mid360_lio_sam_pointcloud_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(
  mid360_lio_sam_pointcloud_adapter_node
  rclcpp
  sensor_msgs
)

add_executable(multi_lidar_pointcloud_fusion_node src/multi_lidar_pointcloud_fusion_node.cpp)
target_compile_features(multi_lidar_pointcloud_fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(
  multi_lidar_pointcloud_fusion_node
  ament_index_cpp
  livox_ros_driver2
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_executable(six_axis_imu_initial_orientation_node src/six_axis_imu_initial_orientation_node.cpp)
target_compile_features(six_axis_imu_initial_orientation_node PUBLIC cxx_std_17)
ament_target_dependencies(
  six_axis_imu_initial_orientation_node
  rclcpp
  sensor_msgs
  tf2
)

install(
  TARGETS
    mid360_lio_sam_pointcloud_adapter_node
    multi_lidar_pointcloud_fusion_node
    six_axis_imu_initial_orientation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
