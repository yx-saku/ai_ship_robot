cmake_minimum_required(VERSION 3.8)
project(ai_ship_robot_description LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

set(gazebo_model_root "${CMAKE_CURRENT_BINARY_DIR}/gazebo_models/ai_ship_robot_description")

install(
  DIRECTORY urdf mesh
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES model.config model.sdf
  DESTINATION share/${PROJECT_NAME}
)

install(CODE [[
  # Gazebo Classic の model path には model だけを見せ、非 model directory の警告を避ける。
  set(model_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/gazebo_models/ai_ship_robot_description")
  set(package_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/ai_ship_robot_description")
  file(MAKE_DIRECTORY "${model_root}")
  file(COPY "${package_root}/model.config" DESTINATION "${model_root}")
  file(COPY "${package_root}/model.sdf" DESTINATION "${model_root}")
  file(CREATE_LINK "${package_root}/mesh" "${model_root}/mesh" SYMBOLIC)
]])

ament_package()
