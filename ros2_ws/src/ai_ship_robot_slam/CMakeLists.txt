cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(ai_ship_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(livox_custommsg_to_pointcloud2_node src/livox_custommsg_to_pointcloud2_node.cpp)
target_compile_features(livox_custommsg_to_pointcloud2_node PUBLIC cxx_std_17)
ament_target_dependencies(
  livox_custommsg_to_pointcloud2_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(livox_custommsg_self_filter_node src/livox_custommsg_self_filter_node.cpp)
target_compile_features(livox_custommsg_self_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(
  livox_custommsg_self_filter_node
  geometry_msgs
  livox_ros_driver2
  rclcpp
  tf2
  tf2_ros
)

add_executable(slam_reference_lidar_static_tf_node src/slam_reference_lidar_static_tf_node.cpp)
target_compile_features(slam_reference_lidar_static_tf_node PUBLIC cxx_std_17)
ament_target_dependencies(
  slam_reference_lidar_static_tf_node
  geometry_msgs
  livox_ros_driver2
  rclcpp
  tf2
  tf2_ros
)

add_executable(pcd_height_diff_map_generator src/pcd_height_diff_map_generator.cpp)
target_compile_features(pcd_height_diff_map_generator PUBLIC cxx_std_17)
add_executable(ground_candidate_map_generator src/ground_candidate_map_generator.cpp)
target_compile_features(ground_candidate_map_generator PUBLIC cxx_std_17)
target_include_directories(ground_candidate_map_generator SYSTEM PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(ground_candidate_map_generator PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(ground_candidate_map_generator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(pcd_localization_node src/pcd_localization_node.cpp)
target_compile_features(pcd_localization_node PUBLIC cxx_std_17)
ament_target_dependencies(
  pcd_localization_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_include_directories(pcd_localization_node SYSTEM PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(pcd_localization_node PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(pcd_localization_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(
  TARGETS
    ground_candidate_map_generator
    livox_custommsg_self_filter_node
    livox_custommsg_to_pointcloud2_node
    pcd_height_diff_map_generator
    pcd_localization_node
    slam_reference_lidar_static_tf_node
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
