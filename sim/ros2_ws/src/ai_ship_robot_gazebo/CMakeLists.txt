cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_gazebo LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(scripted_drive_core src/scripted_drive_core.cpp)
target_compile_features(scripted_drive_core PUBLIC cxx_std_17)
target_include_directories(scripted_drive_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scripted_drive_core yaml-cpp)

add_executable(mid360_sim_adapter src/mid360_sim_adapter.cpp)
target_compile_features(mid360_sim_adapter PRIVATE cxx_std_17)
ament_target_dependencies(mid360_sim_adapter
  ament_index_cpp
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(scripted_drive src/scripted_drive.cpp)
target_compile_features(scripted_drive PRIVATE cxx_std_17)
target_link_libraries(scripted_drive scripted_drive_core)
ament_target_dependencies(scripted_drive
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(keyboard_drive src/keyboard_drive.cpp)
target_compile_features(keyboard_drive PRIVATE cxx_std_17)
ament_target_dependencies(keyboard_drive
  geometry_msgs
  rclcpp
)

add_executable(cmd_vel_slope_adapter src/cmd_vel_slope_adapter.cpp)
target_compile_features(cmd_vel_slope_adapter PRIVATE cxx_std_17)
ament_target_dependencies(cmd_vel_slope_adapter
  geometry_msgs
  rclcpp
)

add_library(cmd_vel_slope_adapter_core OBJECT src/cmd_vel_slope_adapter.cpp)
target_compile_features(cmd_vel_slope_adapter_core PRIVATE cxx_std_17)
target_compile_definitions(cmd_vel_slope_adapter_core PRIVATE AI_SHIP_ROBOT_GAZEBO_BUILDING_TEST=1)
ament_target_dependencies(cmd_vel_slope_adapter_core
  geometry_msgs
  rclcpp
)

add_executable(slope_drive_bridge src/slope_drive_bridge.cpp)
target_compile_features(slope_drive_bridge PRIVATE cxx_std_17)
ament_target_dependencies(slope_drive_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  DIRECTORY config launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    keyboard_drive
    cmd_vel_slope_adapter
    mid360_sim_adapter
    slope_drive_bridge
    scripted_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS scripted_drive_core
  EXPORT scripted_drive_core_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_scripted_drive test/test_scripted_drive.cpp)
  if(TARGET test_scripted_drive)
    target_link_libraries(test_scripted_drive scripted_drive_core)
  endif()

  ament_add_gtest(test_cmd_vel_slope_adapter
    test/test_cmd_vel_slope_adapter.cpp
    $<TARGET_OBJECTS:cmd_vel_slope_adapter_core>
  )
  if(TARGET test_cmd_vel_slope_adapter)
    ament_target_dependencies(test_cmd_vel_slope_adapter
      geometry_msgs
      rclcpp
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(scripted_drive_core_export HAS_LIBRARY_TARGET)

ament_package()
