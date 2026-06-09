cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(dual_lidar_pointcloud_fusion_node src/dual_lidar_pointcloud_fusion_node.cpp)
target_compile_features(dual_lidar_pointcloud_fusion_node PUBLIC cxx_std_17)
target_include_directories(dual_lidar_pointcloud_fusion_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(dual_lidar_pointcloud_fusion_node ${PCL_LIBRARIES})
ament_target_dependencies(
  dual_lidar_pointcloud_fusion_node
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

install(
  TARGETS dual_lidar_pointcloud_fusion_node
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
