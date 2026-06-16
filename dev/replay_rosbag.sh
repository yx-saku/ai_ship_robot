#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
SLAM_RVIZ_CONFIG="${WORKSPACE_ROOT}/ros2_ws/src/ai_ship_robot_slam/rviz/lio_sam.rviz"
SIMULATION_RVIZ_CONFIG="${SIM_ROOT}/ros2_ws/src/ai_ship_robot_gazebo/config/mid360_points.rviz"
ROSBAG_PID=""
RVIZ_PID=""
MAP_SAVER_PID=""
ROSBAG_ROOT="${WORKSPACE_ROOT}/outputs/rosbag2"

usage() {
  cat <<'EOF'
Usage: bash dev/replay_rosbag.sh MODE [BAG_PATH] [OPTIONS]

Arguments:
  MODE                 Required replay mode: sim or slam.
  BAG_PATH             Optional rosbag path. If omitted, the latest outputs/rosbag2/MODE_* bag is used.

Options:
  --rviz-config PATH   Use a custom RViz config.
  --rate VALUE         Set rosbag playback rate. Default: 1.0
  --start-offset SEC   Start playback after the given offset. Default: 0
  --loop               Loop rosbag playback.
  --map                Save accumulated map by calling /save_pcd_map after replay.
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

set_replay_mode() {
  local requested_mode="$1"

  # 収録元に応じてRViz表示内容を切り替えるため、再生モードは明示指定だけを許可する。
  if [[ -n "${REPLAY_MODE}" ]]; then
    echo "Replay mode is already set: ${REPLAY_MODE}" >&2
    exit 2
  fi

  case "${requested_mode}" in
    sim)
      REPLAY_MODE="sim"
      ;;
    slam)
      REPLAY_MODE="slam"
      ;;
    *)
      echo "Unsupported replay mode: ${requested_mode}. Use sim or slam." >&2
      exit 2
      ;;
  esac
}

rviz_config_for_mode() {
  local replay_mode="$1"

  # 既定RViz設定はモードから導出し、従来の単一デフォルトに戻らないようにする。
  case "${replay_mode}" in
    slam)
      printf '%s' "${SLAM_RVIZ_CONFIG}"
      ;;
    sim)
      printf '%s' "${SIMULATION_RVIZ_CONFIG}"
      ;;
  esac
}

latest_bag_for_prefix() {
  local prefix="$1"
  local candidate_metadata=""
  local candidate_dir=""
  local newest_dir=""

  shopt -s nullglob
  for candidate_metadata in "${ROSBAG_ROOT}/${prefix}_"*/metadata.yaml; do
    candidate_dir="${candidate_metadata%/metadata.yaml}"
    if [[ -z "${newest_dir}" || "${candidate_metadata}" -nt "${newest_dir}/metadata.yaml" ]]; then
      newest_dir="${candidate_dir}"
    fi
  done
  shopt -u nullglob

  # metadata.yamlを持つ収録済みbagだけを選び、作成途中ディレクトリを誤って再生しないようにする。
  if [[ -z "${newest_dir}" ]]; then
    echo "No rosbag found for prefix '${prefix}_' in ${ROSBAG_ROOT}." >&2
    exit 1
  fi

  printf '%s' "${newest_dir}"
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
    echo "Run bash install/install_third_party.sh && bash install/setup.sh && bash sim/install/setup.sh." >&2
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
    || ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" \
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
  if [[ -n "${MAP_SAVER_PID}" ]] && kill -0 "${MAP_SAVER_PID}" 2>/dev/null; then
    kill -INT "${MAP_SAVER_PID}" 2>/dev/null || kill -TERM "${MAP_SAVER_PID}" 2>/dev/null || true
    wait "${MAP_SAVER_PID}" 2>/dev/null || true
  fi
}

trap cleanup_background_processes EXIT

REPLAY_MODE=""
BAG_PATH=""
RVIZ_CONFIG=""
PLAY_RATE="1.0"
START_OFFSET="0"
LOOP_PLAYBACK=false
SAVE_MAP=false

if [[ $# -gt 0 && "$1" != "-h" && "$1" != "--help" ]]; then
  if [[ "$1" == "--mode" || "$1" == --mode=* ]]; then
    echo "--mode has been removed. Use the first argument as mode: sim or slam." >&2
    exit 2
  fi
  if [[ "$1" == --* ]]; then
    echo "First argument must be replay mode: sim or slam." >&2
    exit 2
  fi
  set_replay_mode "$1"
  shift

  # 第二引数がoptionでなければ明示bag pathとして扱い、省略時だけprefix最新bagへfallbackする。
  if [[ $# -gt 0 && "$1" != --* ]]; then
    BAG_PATH="$1"
    shift
  fi
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --mode|--mode=*)
      echo "--mode has been removed. Use the first argument as mode: sim or slam." >&2
      exit 2
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
    --map)
      SAVE_MAP=true
      ;;
    *:=*)
      echo "Do not use ROS 2 launch argument syntax here: $1" >&2
      echo "Use shell options instead. Run with --help to see available options." >&2
      exit 2
      ;;
    *)
      echo "Unexpected positional argument: $1" >&2
      echo "Use: bash dev/replay_rosbag.sh MODE [BAG_PATH] [OPTIONS]" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ -z "${REPLAY_MODE}" ]]; then
  echo "Replay mode is required. Use first argument: sim or slam." >&2
  exit 2
fi

if [[ -z "${BAG_PATH}" ]]; then
  BAG_PATH="$(latest_bag_for_prefix "${REPLAY_MODE}")"
  echo "Using latest ${REPLAY_MODE} rosbag: ${BAG_PATH}" >&2
fi

if [[ -z "${RVIZ_CONFIG}" ]]; then
  RVIZ_CONFIG="$(rviz_config_for_mode "${REPLAY_MODE}")"
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
export AI_SHIP_ROBOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}"

PLAY_CMD=(ros2 bag play "${BAG_PATH}" --clock --rate "${PLAY_RATE}" --start-offset "${START_OFFSET}")
if [[ "${LOOP_PLAYBACK}" == "true" ]]; then
  PLAY_CMD+=(--loop)
fi

echo "Replay mode: ${REPLAY_MODE}" >&2
echo "RViz config: ${RVIZ_CONFIG}" >&2
echo "Rosbag replay: ${BAG_PATH}" >&2
if [[ "${SAVE_MAP}" == "true" ]]; then
  ros2 run ai_ship_robot_slam pcd_map_saver_node --ros-args -p use_sim_time:=true &
  MAP_SAVER_PID=$!
  sleep 1
fi
rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!
sleep 2
"${PLAY_CMD[@]}" &
ROSBAG_PID=$!
set +e
wait "${ROSBAG_PID}"
play_status=$?
ROSBAG_PID=""
set -e

if [[ "${play_status}" -ne 0 ]]; then
  exit "${play_status}"
fi

if [[ "${SAVE_MAP}" == "true" ]]; then
  ros2 service call /save_pcd_map std_srvs/srv/Trigger '{}'
fi

# 再生完了後もRVizを残し、結果確認中にwindowが自動で閉じないようにする。
echo "Rosbag playback finished. RViz will remain open; close RViz or press Ctrl-C to exit." >&2
if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
  wait "${RVIZ_PID}"
fi
