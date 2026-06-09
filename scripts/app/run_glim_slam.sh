#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_ENVIRONMENT_SCRIPT="${WORKSPACE_ROOT}/scripts/install/environment.sh"

usage() {
  cat <<'EOF'
Usage: bash scripts/app/run_glim_slam.sh [OPTIONS]

Options:
  --with-sim          Start Gazebo simulation and glim SLAM together.
  --build             Build/install required workspace profile before launch.
  --gui               Enable Gazebo Classic GUI when --with-sim is used.
  --no-gui            Disable Gazebo Classic GUI when --with-sim is used.
  --rviz              Enable RViz2.
  --no-rviz           Disable RViz2.
  --lite              Use lite simulation settings when --with-sim is used.
  --config PATH       Use a glim config directory.
  --left-points TOPIC Set left LiDAR PointCloud2 topic.
  --right-points TOPIC
                      Set right LiDAR PointCloud2 topic.
  --imu-topic TOPIC   Set IMU topic.
  --voxel-leaf SIZE   Set fused cloud voxel leaf size in meters.
  --glim-package NAME Set glim ROS package name.
  --glim-executable NAME
                      Set glim executable name.
  -h, --help          Show this help.
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"

  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "${option} requires a value." >&2
    exit 2
  fi

  printf '%s' "${value}"
}

source_overlay_if_current() {
  local setup_file="$1"

  if [[ ! -f "${setup_file}" ]]; then
    return 0
  fi

  # 古い絶対パスを含むoverlayはsourceせず、再buildを促して環境の混線を避ける。
  if grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"; then
    echo "Stale workspace setup detected: ${setup_file}" >&2
    echo "Run bash scripts/app/run_glim_slam.sh --build or bash scripts/install/environment.sh --workspace-only --profile slam-sim." >&2
    return 1
  fi

  source "${setup_file}"
}

source_workspace_environment() {
  local include_overlays="${1:-true}"
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    return 1
  fi

  # ROS 2本体を先に読み込み、存在するoverlayだけを順番に重ねる。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ "${include_overlays}" == "true" ]]; then
    if ! source_overlay_if_current "${WORKSPACE_ROOT}/third_party/ws/install/setup.bash" \
      || ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash"; then
      if [[ "${had_nounset}" -eq 1 ]]; then
        set -u
      fi
      return 1
    fi
  fi
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

WITH_SIM=false
BUILD_WORKSPACE=false
ROBOT_NAME_SET=false
LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --with-sim)
      WITH_SIM=true
      ;;
    --build)
      BUILD_WORKSPACE=true
      ;;
    --gui)
      LAUNCH_ARGS+=("gui:=true")
      ;;
    --no-gui)
      LAUNCH_ARGS+=("gui:=false")
      ;;
    --rviz)
      LAUNCH_ARGS+=("use_rviz:=true")
      ;;
    --no-rviz)
      LAUNCH_ARGS+=("use_rviz:=false")
      ;;
    --lite)
      LAUNCH_ARGS+=("lite:=true")
      ;;
    --config=*)
      LAUNCH_ARGS+=("glim_config_path:=${1#*=}")
      ;;
    --config)
      shift
      LAUNCH_ARGS+=("glim_config_path:=$(require_value --config "${1:-}")")
      ;;
    --left-points=*)
      LAUNCH_ARGS+=("left_points_topic:=${1#*=}")
      ;;
    --left-points)
      shift
      LAUNCH_ARGS+=("left_points_topic:=$(require_value --left-points "${1:-}")")
      ;;
    --right-points=*)
      LAUNCH_ARGS+=("right_points_topic:=${1#*=}")
      ;;
    --right-points)
      shift
      LAUNCH_ARGS+=("right_points_topic:=$(require_value --right-points "${1:-}")")
      ;;
    --imu-topic=*)
      LAUNCH_ARGS+=("imu_topic:=${1#*=}")
      ;;
    --imu-topic)
      shift
      LAUNCH_ARGS+=("imu_topic:=$(require_value --imu-topic "${1:-}")")
      ;;
    --voxel-leaf=*)
      LAUNCH_ARGS+=("voxel_leaf_size:=${1#*=}")
      ;;
    --voxel-leaf)
      shift
      LAUNCH_ARGS+=("voxel_leaf_size:=$(require_value --voxel-leaf "${1:-}")")
      ;;
    --glim-package=*)
      LAUNCH_ARGS+=("glim_package:=${1#*=}")
      ;;
    --glim-package)
      shift
      LAUNCH_ARGS+=("glim_package:=$(require_value --glim-package "${1:-}")")
      ;;
    --glim-executable=*)
      LAUNCH_ARGS+=("glim_executable:=${1#*=}")
      ;;
    --glim-executable)
      shift
      LAUNCH_ARGS+=("glim_executable:=$(require_value --glim-executable "${1:-}")")
      ;;
    --robot-name=*)
      ROBOT_NAME_SET=true
      LAUNCH_ARGS+=("robot_name:=${1#*=}")
      ;;
    --robot-name)
      shift
      ROBOT_NAME_SET=true
      LAUNCH_ARGS+=("robot_name:=$(require_value --robot-name "${1:-}")")
      ;;
    *:=*)
      echo "Do not use ROS 2 launch argument syntax here: $1" >&2
      echo "Use shell options instead. Run with --help to see available options." >&2
      exit 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Run with --help to see available options." >&2
      exit 2
      ;;
  esac
  shift
done

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  # シミュレーション併用時だけGazebo/Livox依存を含め、実機SLAMではglim中心のprofileにする。
  if [[ "${WITH_SIM}" == "true" ]]; then
    bash "${INSTALL_ENVIRONMENT_SCRIPT}" --workspace-only --profile slam-sim
  else
    bash "${INSTALL_ENVIRONMENT_SCRIPT}" --workspace-only --profile real
  fi
  echo "If the build reports missing apt/CMake dependencies, run scripts/install/environment.sh --system-only with the same profile first." >&2
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash scripts/install/environment.sh --profile slam-sim first." >&2
  exit 1
fi

source_workspace_environment

if [[ "${WITH_SIM}" == "true" ]]; then
  if [[ "${ROBOT_NAME_SET}" == "false" ]]; then
    # Gazeboに前回のentityが残った場合でも、再起動時のspawn名衝突を避ける。
    LAUNCH_ARGS+=("robot_name:=ai_ship_robot_glim_$$")
  fi
  ros2 launch ai_ship_robot_slam sim_glim.launch.py "${LAUNCH_ARGS[@]}"
else
  ros2 launch ai_ship_robot_slam glim.launch.py "${LAUNCH_ARGS[@]}"
fi
