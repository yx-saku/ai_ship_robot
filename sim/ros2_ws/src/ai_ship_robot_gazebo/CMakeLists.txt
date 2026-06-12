cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_gazebo LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/keyboard_drive.py
    scripts/mid360_sim_adapter.py
    scripts/scripted_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_scripted_drive test/test_scripted_drive.py)
endif()

ament_package()
