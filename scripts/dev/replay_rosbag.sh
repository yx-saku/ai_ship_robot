#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
AITRAN_ROOT="${WORKSPACE_ROOT}/aitran"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
DEFAULT_RVIZ_CONFIG="${SIM_ROOT}/ros2_ws/src/ai_ship_robot_gazebo/config/mid360_points.rviz"
ROSBAG_PID=""
RVIZ_PID=""

usage() {
  cat <<'EOF'
Usage: bash scripts/dev/replay_rosbag.sh BAG_PATH [OPTIONS]

Options:
  --rviz-config PATH   Use a custom RViz config.
  --rate VALUE         Set rosbag playback rate. Default: 1.0
  --start-offset SEC   Start playback after the given offset. Default: 0
  --loop               Loop rosbag playback.
  -h, --help           Show this help.
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

  # 既存の workspace 運用と合わせ、古い絶対パスを含む setup は読み込まない。
  if grep -Fq "${WORKSPACE_ROOT}/third_party/ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party/vendor" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"; then
    echo "Stale workspace setup detected: ${setup_file}" >&2
    echo "Run bash aitran/scripts/install/install_third_party.sh && bash aitran/scripts/install/setup.sh && bash sim/scripts/install/setup.sh." >&2
    return 1
  fi

  source "${setup_file}"
}

source_workspace_environment() {
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    return 1
  fi

  # RViz と bag 再生の両方で package 解決できるよう、simulation と slam overlay を重ねる。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if ! source_overlay_if_current "${THIRD_PARTY_UNDERLAY_SETUP}" \
    || ! source_overlay_if_current "${AITRAN_ROOT}/ros2_ws/install/setup.bash" \
    || ! source_overlay_if_current "${SIM_ROOT}/ros2_ws/install/setup.bash"; then
    if [[ "${had_nounset}" -eq 1 ]]; then
      set -u
    fi
    return 1
  fi
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

cleanup_background_processes() {
  # 片方だけ残るのを避けるため、終了時は RViz と bag 再生をまとめて止める。
  if [[ -n "${ROSBAG_PID}" ]] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
    kill -INT "${ROSBAG_PID}" 2>/dev/null || kill -TERM "${ROSBAG_PID}" 2>/dev/null || true
    wait "${ROSBAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill -INT "${RVIZ_PID}" 2>/dev/null || kill -TERM "${RVIZ_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
}

trap cleanup_background_processes EXIT

BAG_PATH=""
RVIZ_CONFIG="${DEFAULT_RVIZ_CONFIG}"
PLAY_RATE="1.0"
START_OFFSET="0"
LOOP_PLAYBACK=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --rviz-config=*)
      RVIZ_CONFIG="${1#*=}"
      ;;
    --rviz-config)
      shift
      RVIZ_CONFIG="$(require_value --rviz-config "${1:-}")"
      ;;
    --rate=*)
      PLAY_RATE="${1#*=}"
      ;;
    --rate)
      shift
      PLAY_RATE="$(require_value --rate "${1:-}")"
      ;;
    --start-offset=*)
      START_OFFSET="${1#*=}"
      ;;
    --start-offset)
      shift
      START_OFFSET="$(require_value --start-offset "${1:-}")"
      ;;
    --loop)
      LOOP_PLAYBACK=true
      ;;
    *:=*)
      echo "Do not use ROS 2 launch argument syntax here: $1" >&2
      echo "Use shell options instead. Run with --help to see available options." >&2
      exit 2
      ;;
    *)
      if [[ -z "${BAG_PATH}" ]]; then
        BAG_PATH="$1"
      else
        echo "Unknown option: $1" >&2
        echo "Run with --help to see available options." >&2
        exit 2
      fi
      ;;
  esac
  shift
done

if [[ -z "${BAG_PATH}" ]]; then
  echo "BAG_PATH is required." >&2
  exit 2
fi

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "Bag path not found: ${BAG_PATH}" >&2
  exit 1
fi

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "RViz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

source_workspace_environment

PLAY_CMD=(ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}" --start-offset "${START_OFFSET}")
if [[ "${LOOP_PLAYBACK}" == "true" ]]; then
  PLAY_CMD+=(--loop)
fi

echo "RViz config: ${RVIZ_CONFIG}" >&2
echo "Rosbag replay: ${BAG_PATH}" >&2
rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!
sleep 2
"${PLAY_CMD[@]}" &
ROSBAG_PID=$!
wait "${ROSBAG_PID}"
