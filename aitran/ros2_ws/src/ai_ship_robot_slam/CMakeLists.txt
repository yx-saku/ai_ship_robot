cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mid360_lio_sam_pointcloud_adapter_node src/mid360_lio_sam_pointcloud_adapter_node.cpp)
target_compile_features(mid360_lio_sam_pointcloud_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(
  mid360_lio_sam_pointcloud_adapter_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS mid360_lio_sam_pointcloud_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
