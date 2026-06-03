#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: bash scripts/run_simulation.sh [OPTIONS]

Options:
  --build             Build ros2_ws before launching simulation.
  --lite              Disable Gazebo GUI and halve LiDAR samples.
  --gui               Enable Gazebo GUI.
  --no-gui            Disable Gazebo GUI.
  --rviz              Enable RViz2.
  --no-rviz           Disable RViz2.
  --half-resolution   Halve horizontal and vertical LiDAR samples.
  --full-resolution   Use full LiDAR sample counts.
  --world PATH        Use a custom Gazebo world.
  --rviz-config PATH  Use a custom RViz config.
  --robot-name NAME   Set the spawned robot name.
  -h, --help          Show this help.

Edit ros2_ws/src/ai_ship_robot_description/urdf/ai_ship_robot.urdf.xacro
to select a LiDAR installation pattern.
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

LAUNCH_ARGS=()
BUILD_WORKSPACE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --build)
      BUILD_WORKSPACE=true
      ;;
    --lite)
      LAUNCH_ARGS+=("lite:=true")
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
    --half-resolution)
      LAUNCH_ARGS+=("half_lidar_resolution:=true")
      ;;
    --full-resolution)
      LAUNCH_ARGS+=("half_lidar_resolution:=false")
      ;;
    --world=*)
      LAUNCH_ARGS+=("world:=${1#*=}")
      ;;
    --world)
      shift
      LAUNCH_ARGS+=("world:=$(require_value --world "${1:-}")")
      ;;
    --rviz-config=*)
      LAUNCH_ARGS+=("rviz_config:=${1#*=}")
      ;;
    --rviz-config)
      shift
      LAUNCH_ARGS+=("rviz_config:=$(require_value --rviz-config "${1:-}")")
      ;;
    --robot-name=*)
      LAUNCH_ARGS+=("robot_name:=${1#*=}")
      ;;
    --robot-name)
      shift
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

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
  colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS}/src" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash scripts/bootstrap_workspace.sh first." >&2
  exit 1
fi

source "${SCRIPT_DIR}/setup_workspace.sh"

ros2 launch ai_ship_robot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}"
