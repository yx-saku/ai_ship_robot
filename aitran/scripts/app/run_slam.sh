#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
AITRAN_ROOT="${WORKSPACE_ROOT}/aitran"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
SETUP_RUNTIME_SCRIPT="${AITRAN_ROOT}/scripts/install/setup.sh"
SETUP_SIMULATION_SCRIPT="${SIM_ROOT}/scripts/install/setup.sh"
LIDAR_PATTERN_DIR="${SIM_ROOT}/ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
FORWARD_ARGS=()
SIM_MODE=false

usage() {
  cat <<'EOF'
Usage: bash aitran/scripts/app/run_slam.sh [OPTIONS]

Options:
  --sim              Launch Gazebo simulation and LIO-SAM together.
  --backend lio-sam  Accepted for compatibility.
  --lio-sam          Accepted for compatibility.
  -h, --help         Show this help.

Examples:
  bash aitran/scripts/app/run_slam.sh --no-rviz
  bash aitran/scripts/app/run_slam.sh --sim --lite --no-gui
  bash aitran/scripts/app/run_slam.sh --points /left_lidar/points --imu /left_lidar/imu
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
    echo "Run bash aitran/scripts/install/install_third_party.sh && bash aitran/scripts/app/run_slam.sh --sim --build." >&2
    return 1
  fi

  source "${setup_file}"
}

source_sim_slam_environment() {
  local include_overlays="${1:-true}"
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    return 1
  fi

  # --sim時だけaitran overlayとsimulation overlayを重ね、sim側にSLAM起動scriptを置かない。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ "${include_overlays}" == "true" ]]; then
    if ! source_overlay_if_current "${THIRD_PARTY_UNDERLAY_SETUP}" \
      || ! source_overlay_if_current "${AITRAN_ROOT}/ros2_ws/install/setup.bash" \
      || ! source_overlay_if_current "${SIM_ROOT}/ros2_ws/install/setup.bash"; then
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

run_sim_lio_sam() {
  local build_workspace=false
  local lite_mode=false
  local robot_name_set=false
  local launch_args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --build)
        build_workspace=true
        ;;
      --config=*)
        launch_args+=("params_file:=${1#*=}")
        ;;
      --config)
        shift
        launch_args+=("params_file:=$(require_value --config "${1:-}")")
        ;;
      --points=*)
        launch_args+=("points_topic:=${1#*=}")
        ;;
      --points)
        shift
        launch_args+=("points_topic:=$(require_value --points "${1:-}")")
        ;;
      --imu=*)
        launch_args+=("imu_topic:=${1#*=}")
        ;;
      --imu)
        shift
        launch_args+=("imu_topic:=$(require_value --imu "${1:-}")")
        ;;
      --lio-points=*)
        launch_args+=("lio_points_topic:=${1#*=}")
        ;;
      --lio-points)
        shift
        launch_args+=("lio_points_topic:=$(require_value --lio-points "${1:-}")")
        ;;
      --adapter)
        launch_args+=("use_adapter:=true")
        ;;
      --no-adapter)
        launch_args+=("use_adapter:=false")
        ;;
      --lidar-frame=*)
        launch_args+=("lidar_frame:=${1#*=}")
        ;;
      --lidar-frame)
        shift
        launch_args+=("lidar_frame:=$(require_value --lidar-frame "${1:-}")")
        ;;
      --base-frame=*)
        launch_args+=("base_frame:=${1#*=}")
        ;;
      --base-frame)
        shift
        launch_args+=("base_frame:=$(require_value --base-frame "${1:-}")")
        ;;
      --odom-frame=*)
        launch_args+=("odom_frame:=${1#*=}")
        ;;
      --odom-frame)
        shift
        launch_args+=("odom_frame:=$(require_value --odom-frame "${1:-}")")
        ;;
      --map-frame=*)
        launch_args+=("map_frame:=${1#*=}")
        ;;
      --map-frame)
        shift
        launch_args+=("map_frame:=$(require_value --map-frame "${1:-}")")
        ;;
      --derived-ring-count=*)
        launch_args+=("derived_ring_count:=${1#*=}")
        ;;
      --derived-ring-count)
        shift
        launch_args+=("derived_ring_count:=$(require_value --derived-ring-count "${1:-}")")
        ;;
      --min-vertical-angle=*)
        launch_args+=("min_vertical_angle_deg:=${1#*=}")
        ;;
      --min-vertical-angle)
        shift
        launch_args+=("min_vertical_angle_deg:=$(require_value --min-vertical-angle "${1:-}")")
        ;;
      --max-vertical-angle=*)
        launch_args+=("max_vertical_angle_deg:=${1#*=}")
        ;;
      --max-vertical-angle)
        shift
        launch_args+=("max_vertical_angle_deg:=$(require_value --max-vertical-angle "${1:-}")")
        ;;
      --rviz)
        launch_args+=("use_rviz:=true")
        ;;
      --no-rviz)
        launch_args+=("use_rviz:=false")
        ;;
      --lite)
        lite_mode=true
        ;;
      --gui)
        launch_args+=("gui:=true")
        ;;
      --no-gui)
        launch_args+=("gui:=false")
        ;;
      --world=*)
        launch_args+=("world:=${1#*=}")
        ;;
      --world)
        shift
        launch_args+=("world:=$(require_value --world "${1:-}")")
        ;;
      --lidar-pattern=*)
        launch_args+=("lidar_pattern_file:=$(validate_lidar_pattern_file "${1#*=}")")
        ;;
      --lidar-pattern)
        shift
        launch_args+=("lidar_pattern_file:=$(validate_lidar_pattern_file "$(require_value --lidar-pattern "${1:-}")")")
        ;;
      --robot-name=*)
        launch_args+=("robot_name:=${1#*=}")
        robot_name_set=true
        ;;
      --robot-name)
        shift
        launch_args+=("robot_name:=$(require_value --robot-name "${1:-}")")
        robot_name_set=true
        ;;
      --publish-map-to-odom-tf)
        launch_args+=("publish_map_to_odom_tf:=true")
        ;;
      --no-publish-map-to-odom-tf)
        launch_args+=("publish_map_to_odom_tf:=false")
        ;;
      --lio-sam-package=*)
        launch_args+=("lio_sam_package:=${1#*=}")
        ;;
      --lio-sam-package)
        shift
        launch_args+=("lio_sam_package:=$(require_value --lio-sam-package "${1:-}")")
        ;;
      *:=*)
        echo "Do not use ROS 2 launch argument syntax here: $1" >&2
        echo "Use shell options instead. Run with --help to see available options." >&2
        exit 2
        ;;
      *)
        echo "Unknown option for --sim: $1" >&2
        echo "Run with --help to see available options." >&2
        exit 2
        ;;
    esac
    shift
  done

  # lite指定時はGazebo GUIを止め、LiDAR処理もsimulation launch側の軽量設定へ切り替える。
  launch_args+=("lite:=${lite_mode}")
  if [[ "${lite_mode}" == "true" ]]; then
    launch_args+=("gui:=false")
  fi

  # Gazebo entity名の衝突を避けるため、未指定時だけ一意な名前を使う。
  if [[ "${robot_name_set}" == "false" ]]; then
    launch_args+=("robot_name:=ai_ship_robot_lio_sam_$$")
  fi

  source_sim_slam_environment false

  if [[ "${build_workspace}" == "true" ]]; then
    # --simのbuildはSLAM workspaceとsimulation workspaceを順に更新する。
    bash "${SETUP_RUNTIME_SCRIPT}"
    bash "${SETUP_SIMULATION_SCRIPT}"
  fi

  if [[ ! -f "${AITRAN_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "Missing aitran/ros2_ws/install/setup.bash. Run bash aitran/scripts/install/setup.sh first." >&2
    exit 1
  fi

  if [[ ! -f "${SIM_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "Missing sim/ros2_ws/install/setup.bash. Run bash sim/scripts/install/setup.sh first." >&2
    exit 1
  fi

  source_sim_slam_environment
  ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --sim)
      SIM_MODE=true
      ;;
    --backend=*)
      if [[ "${1#*=}" != "lio-sam" && "${1#*=}" != "lio_sam" && "${1#*=}" != "liosam" ]]; then
        echo "Only LIO-SAM is supported. Remove --backend or use --backend lio-sam." >&2
        exit 2
      fi
      ;;
    --backend)
      shift
      backend_value="$(require_value --backend "${1:-}")"
      if [[ "${backend_value}" != "lio-sam" && "${backend_value}" != "lio_sam" && "${backend_value}" != "liosam" ]]; then
        echo "Only LIO-SAM is supported. Remove --backend or use --backend lio-sam." >&2
        exit 2
      fi
      ;;
    --lio-sam|--lio_sam|--liosam)
      ;;
    --use-sim-time)
      FORWARD_ARGS+=("$1")
      ;;
    --no-use-sim-time)
      FORWARD_ARGS+=("$1")
      ;;
    *)
      FORWARD_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${SIM_MODE}" == "true" ]]; then
  SIM_ARGS=()
  for arg in "${FORWARD_ARGS[@]}"; do
    case "${arg}" in
      --use-sim-time)
        ;;
      --no-use-sim-time)
        echo "--sim always runs with use_sim_time=true; --no-use-sim-time cannot be used." >&2
        exit 2
        ;;
      *)
        SIM_ARGS+=("${arg}")
        ;;
    esac
  done

  run_sim_lio_sam "${SIM_ARGS[@]}"
  exit $?
fi

# 通常時の汎用入口はLIO-SAM単体起動scriptへの互換ラッパーとして扱う。
exec bash "${SCRIPT_DIR}/run_lio_sam.sh" "${FORWARD_ARGS[@]}"
