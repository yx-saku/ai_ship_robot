#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SETUP_RUNTIME_SCRIPT="${WORKSPACE_ROOT}/scripts/install/setup.sh"
SETUP_SIMULATION_SCRIPT="${WORKSPACE_ROOT}/scripts/install/sim/setup.sh"
LIDAR_PATTERN_DIR="${WORKSPACE_ROOT}/ros2_ws_sim/src/ai_ship_robot_description/urdf/lidar/patterns"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"

print_available_lidar_patterns() {
  local indent="${1:-}"
  local pattern_file=""
  local found_pattern=0

  # LiDAR配置候補は実ファイルから列挙し、help表示と実体のずれを防ぐ。
  for pattern_file in "${LIDAR_PATTERN_DIR}"/lidar_pattern_*.urdf.xacro; do
    [[ -e "${pattern_file}" ]] || continue
    echo "${indent}${pattern_file##*/}"
    found_pattern=1
  done

  if [[ "${found_pattern}" -eq 0 ]]; then
    echo "${indent}(no lidar_pattern_*.urdf.xacro files found in ${LIDAR_PATTERN_DIR})"
  fi
}

usage() {
  cat <<'EOF'
Usage: bash scripts/app/sim/run_glim_slam.sh [OPTIONS]

Options:
  --build             Run required workspace setup before launch.
  --config PATH       Use a glim config directory.
  --left-points TOPIC Set left LiDAR PointCloud2 topic.
  --right-points TOPIC
                      Set right LiDAR PointCloud2 topic.
  --voxel-leaf SIZE   Set fused cloud voxel leaf size in meters.
  --rviz              Enable SLAM RViz2.
  --no-rviz           Disable SLAM RViz2.
  --lite              Disable Gazebo Classic GUI and reduce LiDAR load.
  --gui               Enable Gazebo Classic GUI.
  --no-gui            Disable Gazebo Classic GUI.
  --world PATH        Use a custom Gazebo Classic world.
  --lidar-pattern FILE
                      Use a LiDAR pattern xacro file name.
  --robot-name NAME   Set the spawned robot name.
  --glim-package NAME Set glim ROS package name.
  --glim-executable NAME
                      Set glim executable name.
  -h, --help          Show this help.

Available LiDAR patterns:
EOF
  print_available_lidar_patterns "  "
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

validate_lidar_pattern_file() {
  local file_name="$1"

  # xacroのinclude対象をLiDAR配置ファイル名に限定し、任意パスの読み込みを防ぐ。
  if [[ "${file_name}" == */* || "${file_name}" != lidar_pattern_*.urdf.xacro ]]; then
    echo "Invalid LiDAR pattern file name: ${file_name}" >&2
    echo "Available LiDAR patterns:" >&2
    print_available_lidar_patterns "  " >&2
    exit 2
  fi

  if [[ -f "${LIDAR_PATTERN_DIR}/${file_name}" ]]; then
    printf '%s' "${file_name}"
    return 0
  fi

  # 存在しないパターンはlaunch前に検出し、選択可能なファイル名を表示する。
  echo "Unknown LiDAR pattern file: ${file_name}" >&2
  echo "Available LiDAR patterns:" >&2
  print_available_lidar_patterns "  " >&2
  exit 2
}

source_overlay_if_current() {
  local setup_file="$1"

  if [[ ! -f "${setup_file}" ]]; then
    return 0
  fi

  # colconのsetupファイルはunderlayの絶対パスを持つため、ディレクトリ移動後の古い成果物は再buildする。
  if grep -Fq "${WORKSPACE_ROOT}/third_party/ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party/vendor" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"; then
    echo "Stale workspace setup detected: ${setup_file}" >&2
    echo "Run bash scripts/install/install_third_party.sh && bash scripts/install/sim/install_third_party.sh && bash scripts/app/sim/run_glim_slam.sh --build." >&2
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

  # ROS 2本体を先に読み込み、production overlayの上にsimulation overlayを重ねる。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ "${include_overlays}" == "true" ]]; then
    if ! source_overlay_if_current "${THIRD_PARTY_UNDERLAY_SETUP}" \
      || ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" \
      || ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws_sim/install/setup.bash"; then
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

BUILD_WORKSPACE=false
LITE_MODE=false
ROBOT_NAME_SET=false
LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --build)
      BUILD_WORKSPACE=true
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
    --voxel-leaf=*)
      LAUNCH_ARGS+=("voxel_leaf_size:=${1#*=}")
      ;;
    --voxel-leaf)
      shift
      LAUNCH_ARGS+=("voxel_leaf_size:=$(require_value --voxel-leaf "${1:-}")")
      ;;
    --rviz)
      LAUNCH_ARGS+=("use_rviz:=true")
      ;;
    --no-rviz)
      LAUNCH_ARGS+=("use_rviz:=false")
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
    --world=*)
      LAUNCH_ARGS+=("world:=${1#*=}")
      ;;
    --world)
      shift
      LAUNCH_ARGS+=("world:=$(require_value --world "${1:-}")")
      ;;
    --lidar-pattern=*)
      LAUNCH_ARGS+=("lidar_pattern_file:=$(validate_lidar_pattern_file "${1#*=}")")
      ;;
    --lidar-pattern)
      shift
      LAUNCH_ARGS+=("lidar_pattern_file:=$(validate_lidar_pattern_file "$(require_value --lidar-pattern "${1:-}")")")
      ;;
    --robot-name=*)
      LAUNCH_ARGS+=("robot_name:=${1#*=}")
      ROBOT_NAME_SET=true
      ;;
    --robot-name)
      shift
      LAUNCH_ARGS+=("robot_name:=$(require_value --robot-name "${1:-}")")
      ROBOT_NAME_SET=true
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

# lite指定時はGazebo GUIを止め、LiDAR処理もsimulation launch側の軽量設定へ切り替える。
LAUNCH_ARGS+=("lite:=${LITE_MODE}")
if [[ "${LITE_MODE}" == "true" ]]; then
  LAUNCH_ARGS+=("gui:=false")
fi

# Gazebo entity名の衝突を避けるため、未指定時だけ一意な名前を使う。
if [[ "${ROBOT_NAME_SET}" == "false" ]]; then
  LAUNCH_ARGS+=("robot_name:=ai_ship_robot_glim_$$")
fi

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  # simulation起動時だけsimulation workspace setupも実行し、production scriptには持ち込まない。
  bash "${SETUP_RUNTIME_SCRIPT}"
  bash "${SETUP_SIMULATION_SCRIPT}"
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash scripts/install/setup.sh first." >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws_sim/install/setup.bash" ]]; then
  echo "Missing ros2_ws_sim/install/setup.bash. Run bash scripts/install/sim/setup.sh first." >&2
  exit 1
fi

source_workspace_environment

ros2 launch ai_ship_robot_gazebo sim_glim.launch.py "${LAUNCH_ARGS[@]}"
