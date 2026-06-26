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

install(
  DIRECTORY config launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    keyboard_drive
    mid360_sim_adapter
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
endif()

ament_export_include_directories(include)
ament_export_targets(scripted_drive_core_export HAS_LIBRARY_TARGET)

ament_package()
