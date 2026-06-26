cmake_minimum_required(VERSION 3.12)
project(ai_ship_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_library(map_saver_utils STATIC src/map_saver_utils.cpp)
target_compile_features(map_saver_utils PUBLIC cxx_std_17)
target_include_directories(map_saver_utils PUBLIC include)

add_executable(map_saver_node src/map_saver_node.cpp)
target_compile_features(map_saver_node PUBLIC cxx_std_17)
ament_target_dependencies(
  map_saver_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(map_saver_node map_saver_utils)

add_executable(pcd_height_diff_map_generator src/pcd_height_diff_map_generator.cpp)
target_compile_features(pcd_height_diff_map_generator PUBLIC cxx_std_17)
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
    livox_custommsg_self_filter_node
    livox_custommsg_to_pointcloud2_node
    map_saver_node
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
  add_executable(map_saver_utils_test test/map_saver_utils_test.cpp)
  target_compile_features(map_saver_utils_test PUBLIC cxx_std_17)
  target_include_directories(map_saver_utils_test SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(map_saver_utils_test PUBLIC ${PCL_DEFINITIONS})
  target_link_libraries(map_saver_utils_test map_saver_utils ${PCL_LIBRARIES})
  add_test(NAME map_saver_utils_test COMMAND map_saver_utils_test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
