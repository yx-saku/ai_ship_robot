#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: bash scripts/drive_robot.sh [OPTIONS]

Options:
  --build                  Run one-time environment setup before starting keyboard drive.
  --linear-speed VALUE     Forward/backward speed in m/s. Default: 0.20
  --lateral-speed VALUE    Left/right strafe speed in m/s. Default: 0.20
  --angular-speed VALUE    Yaw speed in rad/s. Default: 0.60
  --publish-rate VALUE     Command publish rate in Hz. Default: 10.0
  --cmd-vel-topic TOPIC    cmd_vel topic. Default: cmd_vel
  -h, --help               Show this help.

Keys:
  w/i: toggle forward, s/,: toggle backward
  j/l: toggle strafe left/right, a/d: toggle yaw left/right
  Combine keys sequentially, e.g. w then a for a forward-left arc.
  Press the same component key again to clear only that component.
  space/x/k: stop all, Q/Esc: quit

This script only publishes cmd_vel. Start Gazebo/LiDAR separately with:
  bash scripts/run_simulation.sh
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

cleanup_terminal() {
  if [[ -t 0 ]]; then
    stty sane 2>/dev/null || true
  fi
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
    echo "Run bash scripts/drive_robot.sh --build or bash scripts/install_environment.sh --workspace-only." >&2
    return 1
  fi

  source "${setup_file}"
}

trap cleanup_terminal EXIT

BUILD_WORKSPACE=false
LINEAR_SPEED="0.20"
LATERAL_SPEED="0.20"
ANGULAR_SPEED="0.60"
PUBLISH_RATE="10.0"
CMD_VEL_TOPIC="cmd_vel"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --build)
      BUILD_WORKSPACE=true
      ;;
    --linear-speed=*)
      LINEAR_SPEED="${1#*=}"
      ;;
    --linear-speed)
      shift
      LINEAR_SPEED="$(require_value --linear-speed "${1:-}")"
      ;;
    --lateral-speed=*)
      LATERAL_SPEED="${1#*=}"
      ;;
    --lateral-speed)
      shift
      LATERAL_SPEED="$(require_value --lateral-speed "${1:-}")"
      ;;
    --angular-speed=*)
      ANGULAR_SPEED="${1#*=}"
      ;;
    --angular-speed)
      shift
      ANGULAR_SPEED="$(require_value --angular-speed "${1:-}")"
      ;;
    --publish-rate=*)
      PUBLISH_RATE="${1#*=}"
      ;;
    --publish-rate)
      shift
      PUBLISH_RATE="$(require_value --publish-rate "${1:-}")"
      ;;
    --cmd-vel-topic=*)
      CMD_VEL_TOPIC="${1#*=}"
      ;;
    --cmd-vel-topic)
      shift
      CMD_VEL_TOPIC="$(require_value --cmd-vel-topic "${1:-}")"
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
  bash "${SCRIPT_DIR}/install_environment.sh" --workspace-only
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash scripts/install_environment.sh first." >&2
  exit 1
fi

source_workspace_environment

DRIVE_EXECUTABLE="$(ros2 pkg prefix ai_ship_robot_gazebo)/lib/ai_ship_robot_gazebo/keyboard_drive.py"
if [[ ! -x "${DRIVE_EXECUTABLE}" ]]; then
  echo "Missing keyboard_drive executable: ${DRIVE_EXECUTABLE}" >&2
  echo "Run bash scripts/drive_robot.sh --build or bash scripts/install_environment.sh first." >&2
  exit 1
fi

"${DRIVE_EXECUTABLE}" --ros-args \
  -p cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  -p linear_speed:="${LINEAR_SPEED}" \
  -p lateral_speed:="${LATERAL_SPEED}" \
  -p angular_speed:="${ANGULAR_SPEED}" \
  -p publish_rate:="${PUBLISH_RATE}"
