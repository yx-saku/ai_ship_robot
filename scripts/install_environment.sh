#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
THIRD_PARTY_WS="${WORKSPACE_ROOT}/third_party_ws"
THIRD_PARTY_SRC_DIR="${THIRD_PARTY_WS}/src"
LIVOX_DRIVER_DIR="${THIRD_PARTY_SRC_DIR}/livox_ros_driver2"
LIVOX_SIM_DIR="${THIRD_PARTY_SRC_DIR}/ros2_livox_simulation"
LIVOX_SDK_DIR="${WORKSPACE_ROOT}/third_party_vendor/Livox-SDK2"
INSTALL_MODE="full"
export DEBIAN_FRONTEND

usage() {
  cat <<'EOF'
Usage: bash scripts/install_environment.sh [OPTIONS]

Options:
  --full           追加依存を入れ、外部repo取得とworkspace buildまで実行する。既定値。
  --system-only    apt/rosdep依存だけを入れる。Docker image build用。
  --workspace-only 外部repo取得とworkspace buildだけを実行する。開発コンテナ初回セットアップ用。
  -h, --help       このhelpを表示する。

ROS 2 Humbleは /opt/ros/${ROS_DISTRO} にインストール済みであること。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --full)
      INSTALL_MODE="full"
      ;;
    --system-only)
      INSTALL_MODE="system-only"
      ;;
    --workspace-only)
      INSTALL_MODE="workspace-only"
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Run with --help to see available options." >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required when running as a non-root user." >&2
    exit 1
  fi
  SUDO="sudo"
fi

require_ros2() {
  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} before running this script." >&2
    exit 1
  fi
}

source_ros2() {
  # ROS setup scriptはnounset-safeではないため、読み込み中だけnounsetを解除する。
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

install_system_dependencies() {
  local system_packages=(
    build-essential
    ca-certificates
    cmake
    curl
    gazebo
    git
    libapr1-dev
    libaprutil1-dev
    libboost-chrono-dev
    libboost-dev
    libeigen3-dev
    libgazebo-dev
    libpcl-dev
    libprotobuf-dev
    locales
    protobuf-compiler
    python3-colcon-common-extensions
    python3-rosdep
    sudo
    tzdata
  )
  local ros_packages=(
    "ros-${ROS_DISTRO}-ament-cmake"
    "ros-${ROS_DISTRO}-ament-cmake-auto"
    "ros-${ROS_DISTRO}-diagnostic-msgs"
    "ros-${ROS_DISTRO}-gazebo-dev"
    "ros-${ROS_DISTRO}-gazebo-plugins"
    "ros-${ROS_DISTRO}-gazebo-ros"
    "ros-${ROS_DISTRO}-launch"
    "ros-${ROS_DISTRO}-launch-ros"
    "ros-${ROS_DISTRO}-message-filters"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-robot-state-publisher"
    "ros-${ROS_DISTRO}-ros2launch"
    "ros-${ROS_DISTRO}-ros2run"
    "ros-${ROS_DISTRO}-ros2service"
    "ros-${ROS_DISTRO}-ros2topic"
    "ros-${ROS_DISTRO}-rosbag2"
    "ros-${ROS_DISTRO}-rviz2"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-tf2-sensor-msgs"
    "ros-${ROS_DISTRO}-xacro"
  )

  # 本番Jetsonと開発Dockerで同じ依存リストを使い、ROS 2本体以外の追加packageだけを導入する。
  ${SUDO} apt-get update
  ${SUDO} apt-get install -y --no-install-recommends "${system_packages[@]}" "${ros_packages[@]}"

  if command -v locale-gen >/dev/null 2>&1; then
    ${SUDO} locale-gen en_US en_US.UTF-8 ja_JP.UTF-8
    ${SUDO} update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  fi

  if command -v rosdep >/dev/null 2>&1; then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
      ${SUDO} rosdep init || true
    fi
    rosdep update
  fi
}

ensure_git_repo() {
  local target_dir="$1"
  local repo_url="$2"

  if [[ -d "${target_dir}/.git" ]]; then
    return 0
  fi

  mkdir -p "$(dirname "${target_dir}")"
  git clone --depth 1 "${repo_url}" "${target_dir}"
}

patch_ros2_livox_simulation_repo() {
  local target_dir="$1"
  local marker_file="$2"

  if [[ -f "${marker_file}" ]]; then
    return 0
  fi

  # Jammy/Humbleでのビルド互換と既存topic名維持に必要な最小差分だけを適用する。
  TARGET_DIR="${target_dir}" python3 <<'PY'
from pathlib import Path
import os

target_dir = Path(os.environ["TARGET_DIR"])

cmake_path = target_dir / "CMakeLists.txt"
cmake_text = cmake_path.read_text(encoding="utf-8")
cmake_text = cmake_text.replace(
    "find_package(ament_cmake REQUIRED)\nfind_package(rclcpp REQUIRED)",
    "find_package(ament_cmake REQUIRED)\nfind_package(Protobuf REQUIRED)\nfind_package(rclcpp REQUIRED)",
)
cmake_text = cmake_text.replace(
    "find_package(tf2_ros REQUIRED)\nfind_package(geometry_msgs REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)",
    "find_package(tf2_ros REQUIRED)",
)
cmake_text = cmake_text.replace("include_directories(/usr/include/gazebo-11/gazebo)\n", "")
cmake_text = cmake_text.replace(
    "target_link_libraries(ros2_livox ${GAZEBO_LIBRARIES} RayPlugin GpuRayPlugin)\n"
    "ament_target_dependencies(ros2_livox rclcpp std_msgs sensor_msgs geometry_msgs gazebo_dev gazebo_ros tf2_ros livox_ros_driver2 )\n"
    "target_link_libraries(ros2_livox libprotobuf.so.9)\n"
    "target_link_libraries(ros2_livox libboost_chrono.so.1.71.0)",
    "target_link_libraries(ros2_livox ${GAZEBO_LIBRARIES} RayPlugin GpuRayPlugin protobuf::libprotobuf Boost::chrono)\n"
    "ament_target_dependencies(ros2_livox rclcpp std_msgs sensor_msgs geometry_msgs gazebo_dev gazebo_ros tf2_ros livox_ros_driver2 )",
)
cmake_path.write_text(cmake_text, encoding="utf-8")

plugin_path = target_dir / "src" / "livox_points_plugin.cpp"
plugin_text = plugin_path.read_text(encoding="utf-8")
plugin_text = plugin_text.replace(
    "        auto curr_scan_topic = sdf->Get<std::string>(\"topic\");\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"ros topic name: %s\", curr_scan_topic.c_str());",
    "        auto curr_scan_topic = sdf->Get<std::string>(\"topic\");\n"
    "        auto custom_topic = curr_scan_topic;\n"
    "        auto pointcloud2_topic = curr_scan_topic + \"_PointCloud2\";\n"
    "        if (sdf->HasElement(\"custom_topic\"))\n"
    "        {\n"
    "            custom_topic = sdf->Get<std::string>(\"custom_topic\");\n"
    "        }\n"
    "        if (sdf->HasElement(\"pointcloud2_topic\"))\n"
    "        {\n"
    "            pointcloud2_topic = sdf->Get<std::string>(\"pointcloud2_topic\");\n"
    "        }\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"custom topic name: %s\", custom_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"pointcloud2 topic name: %s\", pointcloud2_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"ros topic name: %s\", curr_scan_topic.c_str());",
)
plugin_text = plugin_text.replace(
    "        // PointCloud2 publisher\n"
    "        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + \"_PointCloud2\", 10);\n"
    "        // CustomMsg publisher\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);",
    "        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud2_topic, 10);\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(custom_topic, 10);",
)
plugin_text = plugin_text.replace(
    "            // Fill the PointCloud point cloud message\n"
    "            clouds.emplace_back();\n"
    "            clouds.back().x = point.X();\n"
    "            clouds.back().y = point.Y();\n"
    "            clouds.back().z = point.Z();\n\n"
    "            // Fill the PointCloud point cloud message\n",
    "            // Fill the PointCloud point cloud message\n",
    1,
)
plugin_path.write_text(plugin_text, encoding="utf-8")
PY

  touch "${marker_file}"
}

prepare_livox_ros_driver2_manifest() {
  local ros2_manifest="${LIVOX_DRIVER_DIR}/package_ROS2.xml"
  local active_manifest="${LIVOX_DRIVER_DIR}/package.xml"

  if [[ ! -f "${ros2_manifest}" ]]; then
    echo "Missing ROS2 manifest: ${ros2_manifest}" >&2
    return 1
  fi

  # upstream build.shと同じmanifest切替を先に行い、rosdepが依存を読めるようにする。
  cp "${ros2_manifest}" "${active_manifest}"
}

install_livox_sdk2_if_needed() {
  local sdk_build_dir="${LIVOX_SDK_DIR}/build"
  local livox_sdk_cxx_flags="${CMAKE_CXX_FLAGS:-}"

  if [[ -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]]; then
    return 0
  fi

  if [[ ! -d "${LIVOX_SDK_DIR}" ]]; then
    echo "Missing Livox-SDK2 repository at ${LIVOX_SDK_DIR}" >&2
    return 1
  fi

  # Livox-SDK2同梱RapidJSONの古いpragma抑制記述がGCCで大量の-Wpragmasログを出すため、SDKソースを変更せずビルド時だけ抑制する。
  if [[ " ${livox_sdk_cxx_flags} " != *" -Wno-pragmas "* ]]; then
    livox_sdk_cxx_flags="${livox_sdk_cxx_flags:+${livox_sdk_cxx_flags} }-Wno-pragmas"
  fi

  # 公式driverが要求する共有SDKを先に入れ、upstream build.shのlink失敗を防ぐ。
  cmake -S "${LIVOX_SDK_DIR}" -B "${sdk_build_dir}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="${livox_sdk_cxx_flags}"
  cmake --build "${sdk_build_dir}" --parallel
  ${SUDO} cmake --install "${sdk_build_dir}"
  ${SUDO} ldconfig
}

install_rosdeps_for_workspace() {
  local workspace_src="$1"

  if ! command -v rosdep >/dev/null 2>&1; then
    return 0
  fi

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    ${SUDO} rosdep init || true
  fi

  rosdep update
  rosdep install --from-paths "${workspace_src}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

clone_external_repositories() {
  # 外部repoはgit管理外の専用領域に置き、upstreamの構成をできるだけ維持する。
  ensure_git_repo "${LIVOX_DRIVER_DIR}" "https://github.com/Livox-SDK/livox_ros_driver2.git"
  ensure_git_repo "${LIVOX_SIM_DIR}" "https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git"
  ensure_git_repo "${LIVOX_SDK_DIR}" "https://github.com/Livox-SDK/Livox-SDK2.git"
  patch_ros2_livox_simulation_repo "${LIVOX_SIM_DIR}" "${LIVOX_SIM_DIR}/.ai_ship_robot_patch_applied"
}

build_third_party_workspace() {
  if [[ ! -d "${LIVOX_DRIVER_DIR}" ]]; then
    echo "Missing livox_ros_driver2 repository at ${LIVOX_DRIVER_DIR}" >&2
    return 1
  fi

  # 公式repoの想定手順を優先し、driver自身のbuild scriptでthird-party workspaceを構築する。
  bash "${LIVOX_DRIVER_DIR}/build.sh" humble
}

build_project_workspace() {
  # install/配下に削除済みpackageが残るのを避けるため、毎回clean buildする。
  rm -rf "${ROS_WS}/build" "${ROS_WS}/install" "${ROS_WS}/log"
  colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS}/src" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
}

install_workspace() {
  source_ros2
  clone_external_repositories
  install_livox_sdk2_if_needed
  prepare_livox_ros_driver2_manifest
  install_rosdeps_for_workspace "${THIRD_PARTY_SRC_DIR}"
  build_third_party_workspace

  # third_party overlayを重ねてからプロジェクトworkspaceのrosdep解決とビルドを行う。
  set +u
  source "${THIRD_PARTY_WS}/install/setup.bash"
  set -u
  install_rosdeps_for_workspace "${ROS_WS}/src"
  build_project_workspace
}

require_ros2

case "${INSTALL_MODE}" in
  full)
    install_system_dependencies
    install_workspace
    ;;
  system-only)
    install_system_dependencies
    ;;
  workspace-only)
    install_workspace
    ;;
esac
