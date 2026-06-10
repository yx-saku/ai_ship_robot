cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_description LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
