cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_gazebo LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/keyboard_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
