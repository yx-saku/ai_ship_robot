#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_ENVIRONMENT_SCRIPT="${WORKSPACE_ROOT}/scripts/install/environment.sh"

usage() {
  cat <<'EOF'
Usage: bash scripts/app/run_simulation.sh [OPTIONS]

Options:
  --build             Run one-time environment setup before launching simulation.
  --lite              Disable Gazebo Classic GUI and default LiDAR rays to quarter resolution.
  --gui               Enable Gazebo Classic GUI.
  --no-gui            Disable Gazebo Classic GUI.
  --rviz              Enable RViz2.
  --no-rviz           Disable RViz2.
  --quarter-resolution Use quarter LiDAR sample density.
  --half-resolution   Use half LiDAR sample density.
  --full-resolution   Use full LiDAR sample counts.
  --world PATH        Use a custom Gazebo Classic world.
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

source_workspace_environment() {
  local include_overlays="${1:-true}"
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    return 1
  fi

  # ROS 2本体を先に読み込み、存在するoverlayだけを順番に重ねて実行時環境を作る。
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

source_overlay_if_current() {
  local setup_file="$1"

  if [[ ! -f "${setup_file}" ]]; then
    return 0
  fi

  # colconのsetupファイルはunderlayの絶対パスを持つため、ディレクトリ移動後の古い成果物は再buildする。
  if grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"; then
    echo "Stale workspace setup detected: ${setup_file}" >&2
    echo "Run bash scripts/app/run_simulation.sh --build or bash scripts/install/environment.sh --workspace-only." >&2
    return 1
  fi

  source "${setup_file}"
}

LAUNCH_ARGS=()
BUILD_WORKSPACE=false
LITE_MODE=false
LIDAR_RESOLUTION_MODE="default"

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
      LITE_MODE=true
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
    --quarter-resolution)
      LIDAR_RESOLUTION_MODE="quarter"
      ;;
    --half-resolution)
      LIDAR_RESOLUTION_MODE="half"
      ;;
    --full-resolution)
      LIDAR_RESOLUTION_MODE="full"
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

# lite指定時は1/4解像度を既定値にし、個別オプションがあればそちらを優先する。
LAUNCH_ARGS+=("lite:=${LITE_MODE}")
case "${LIDAR_RESOLUTION_MODE}" in
  quarter)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=true")
    ;;
  half)
    LAUNCH_ARGS+=("half_lidar_resolution:=true" "quarter_lidar_resolution:=false")
    ;;
  full)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=false")
    ;;
  default)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=false")
    ;;
esac

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  bash "${INSTALL_ENVIRONMENT_SCRIPT}" --workspace-only
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash scripts/install/environment.sh first." >&2
  exit 1
fi

source_workspace_environment

ros2 launch ai_ship_robot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}"
