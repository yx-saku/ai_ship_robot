#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
AITRAN_ROOT="${WORKSPACE_ROOT}/aitran"
SETUP_RUNTIME_SCRIPT="${AITRAN_ROOT}/scripts/install/setup.sh"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"

usage() {
  cat <<'EOF'
Usage: bash aitran/scripts/app/run_lio_sam.sh [OPTIONS]

Options:
  --build                     Build/install required workspace profile before launch.
  --config PATH               Use a LIO-SAM params YAML file.
  --fusion-config PATH        Use a multi-LiDAR fusion params YAML file.
  --input-points CSV          Set input PointCloud2 topics as comma-separated list.
  --left-points TOPIC         Compatibility alias for adding / replacing left LiDAR CustomMsg input.
  --right-points TOPIC        Compatibility alias for adding / replacing right LiDAR CustomMsg input.
  --reference-points TOPIC    Set reference LiDAR PointCloud2 topic.
  --reference-lidar-frame FRAME
                              Set reference LiDAR frame used for fusion output.
  --fused-points TOPIC        Set fused PointCloud2 topic before adapter.
  --imu TOPIC                 Set reference LiDAR IMU topic.
  --lio-points TOPIC          Set adapted PointCloud2 topic passed to LIO-SAM.
  --adapter                   Enable Mid-360 PointCloud2 adapter. Default.
  --no-adapter                Disable adapter and pass --lio-points directly to LIO-SAM.
  --lidar-frame FRAME         Set LIO-SAM lidar frame.
  --base-frame FRAME          Set LIO-SAM base frame.
  --odom-frame FRAME          Set LIO-SAM odom frame.
  --map-frame FRAME           Set LIO-SAM map frame.
  --derived-ring-count N      Set pseudo ring count when raw cloud has no ring/line field.
  --min-vertical-angle DEG    Set minimum vertical angle for pseudo ring derivation.
  --max-vertical-angle DEG    Set maximum vertical angle for pseudo ring derivation.
  --fusion-timestamp-scale S  Set integer point timestamp unit scale for fusion.
  --use-sim-time              Use ROS simulation time for Gazebo or rosbag playback.
  --no-use-sim-time           Use system time. Default.
  --rviz                      Enable RViz2.
  --no-rviz                   Disable RViz2.
  --publish-map-to-odom-tf    Publish static map -> odom TF. Default.
  --no-publish-map-to-odom-tf Disable static map -> odom TF.
  --lio-sam-package NAME      Set LIO-SAM ROS package name.
  -h, --help                  Show this help.
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

  # 古い絶対パスを含むoverlayはsourceせず、system underlayへの移行漏れを明示する。
  if grep -Fq "${WORKSPACE_ROOT}/third_party/ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party/vendor" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"; then
    echo "Stale workspace setup detected: ${setup_file}" >&2
    echo "Run bash aitran/scripts/install/install_third_party.sh && bash aitran/scripts/app/run_lio_sam.sh --build." >&2
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

  # ROS 2本体、third_party underlay、aitran workspaceの順でsourceし、LIO-SAM packageを解決する。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ "${include_overlays}" == "true" ]]; then
    if ! source_overlay_if_current "${THIRD_PARTY_UNDERLAY_SETUP}"; then
      if [[ "${had_nounset}" -eq 1 ]]; then
        set -u
      fi
      return 1
    fi
    if ! source_overlay_if_current "${AITRAN_ROOT}/ros2_ws/install/setup.bash"; then
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
LAUNCH_ARGS=()
LEFT_POINTS_TOPIC=""
RIGHT_POINTS_TOPIC=""

format_topic_list() {
  local values=("$@")
  local result="["
  local index=0

  for value in "${values[@]}"; do
    [[ -n "${value}" ]] || continue
    if [[ "${index}" -gt 0 ]]; then
      result+=", "
    fi
    result+="'${value}'"
    index=$((index + 1))
  done

  result+="]"
  printf '%s' "${result}"
}

parse_csv_topics() {
  local csv="$1"
  local topic=""
  local topics=()

  IFS=',' read -r -a topics <<< "${csv}"
  for topic in "${topics[@]}"; do
    topic="${topic//[[:space:]]/}"
    [[ -n "${topic}" ]] && printf '%s\n' "${topic}"
  done
}

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
      LAUNCH_ARGS+=("params_file:=${1#*=}")
      ;;
    --config)
      shift
      LAUNCH_ARGS+=("params_file:=$(require_value --config "${1:-}")")
      ;;
    --fusion-config=*)
      LAUNCH_ARGS+=("fusion_config:=${1#*=}")
      ;;
    --fusion-config)
      shift
      LAUNCH_ARGS+=("fusion_config:=$(require_value --fusion-config "${1:-}")")
      ;;
    --input-points=*)
      mapfile -t input_topics < <(parse_csv_topics "${1#*=}")
      if [[ "${#input_topics[@]}" -eq 0 ]]; then
        echo "--input-points requires at least one topic." >&2
        exit 2
      fi
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
      ;;
    --input-points)
      shift
      input_points_value="$(require_value --input-points "${1:-}")"
      mapfile -t input_topics < <(parse_csv_topics "${input_points_value}")
      if [[ "${#input_topics[@]}" -eq 0 ]]; then
        echo "--input-points requires at least one topic." >&2
        exit 2
      fi
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
      ;;
    --points=*)
      points_topic="${1#*=}"
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${points_topic}")")
      LAUNCH_ARGS+=("reference_points_topic:=${points_topic}")
      ;;
    --points)
      shift
      points_topic="$(require_value --points "${1:-}")"
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${points_topic}")")
      LAUNCH_ARGS+=("reference_points_topic:=${points_topic}")
      ;;
    --left-points=*)
      LEFT_POINTS_TOPIC="${1#*=}"
      ;;
    --left-points)
      shift
      LEFT_POINTS_TOPIC="$(require_value --left-points "${1:-}")"
      ;;
    --right-points=*)
      RIGHT_POINTS_TOPIC="${1#*=}"
      ;;
    --right-points)
      shift
      RIGHT_POINTS_TOPIC="$(require_value --right-points "${1:-}")"
      ;;
    --reference-points=*)
      LAUNCH_ARGS+=("reference_points_topic:=${1#*=}")
      ;;
    --reference-points)
      shift
      LAUNCH_ARGS+=("reference_points_topic:=$(require_value --reference-points "${1:-}")")
      ;;
    --reference-lidar-frame=*)
      LAUNCH_ARGS+=("reference_lidar_frame:=${1#*=}")
      ;;
    --reference-lidar-frame)
      shift
      LAUNCH_ARGS+=("reference_lidar_frame:=$(require_value --reference-lidar-frame "${1:-}")")
      ;;
    --fused-points=*)
      LAUNCH_ARGS+=("fused_points_topic:=${1#*=}")
      ;;
    --fused-points)
      shift
      LAUNCH_ARGS+=("fused_points_topic:=$(require_value --fused-points "${1:-}")")
      ;;
    --imu=*)
      LAUNCH_ARGS+=("imu_topic:=${1#*=}")
      ;;
    --imu)
      shift
      LAUNCH_ARGS+=("imu_topic:=$(require_value --imu "${1:-}")")
      ;;
    --lio-points=*)
      LAUNCH_ARGS+=("lio_points_topic:=${1#*=}")
      ;;
    --lio-points)
      shift
      LAUNCH_ARGS+=("lio_points_topic:=$(require_value --lio-points "${1:-}")")
      ;;
    --adapter)
      LAUNCH_ARGS+=("use_adapter:=true")
      ;;
    --no-adapter)
      LAUNCH_ARGS+=("use_adapter:=false")
      ;;
    --lidar-frame=*)
      LAUNCH_ARGS+=("lidar_frame:=${1#*=}")
      ;;
    --lidar-frame)
      shift
      LAUNCH_ARGS+=("lidar_frame:=$(require_value --lidar-frame "${1:-}")")
      ;;
    --base-frame=*)
      LAUNCH_ARGS+=("base_frame:=${1#*=}")
      ;;
    --base-frame)
      shift
      LAUNCH_ARGS+=("base_frame:=$(require_value --base-frame "${1:-}")")
      ;;
    --odom-frame=*)
      LAUNCH_ARGS+=("odom_frame:=${1#*=}")
      ;;
    --odom-frame)
      shift
      LAUNCH_ARGS+=("odom_frame:=$(require_value --odom-frame "${1:-}")")
      ;;
    --map-frame=*)
      LAUNCH_ARGS+=("map_frame:=${1#*=}")
      ;;
    --map-frame)
      shift
      LAUNCH_ARGS+=("map_frame:=$(require_value --map-frame "${1:-}")")
      ;;
    --derived-ring-count=*)
      LAUNCH_ARGS+=("derived_ring_count:=${1#*=}")
      ;;
    --derived-ring-count)
      shift
      LAUNCH_ARGS+=("derived_ring_count:=$(require_value --derived-ring-count "${1:-}")")
      ;;
    --min-vertical-angle=*)
      LAUNCH_ARGS+=("min_vertical_angle_deg:=${1#*=}")
      ;;
    --min-vertical-angle)
      shift
      LAUNCH_ARGS+=("min_vertical_angle_deg:=$(require_value --min-vertical-angle "${1:-}")")
      ;;
    --max-vertical-angle=*)
      LAUNCH_ARGS+=("max_vertical_angle_deg:=${1#*=}")
      ;;
    --max-vertical-angle)
      shift
      LAUNCH_ARGS+=("max_vertical_angle_deg:=$(require_value --max-vertical-angle "${1:-}")")
      ;;
    --fusion-timestamp-scale=*)
      LAUNCH_ARGS+=("fusion_timestamp_unit_scale:=${1#*=}")
      ;;
    --fusion-timestamp-scale)
      shift
      LAUNCH_ARGS+=("fusion_timestamp_unit_scale:=$(require_value --fusion-timestamp-scale "${1:-}")")
      ;;
    --use-sim-time)
      LAUNCH_ARGS+=("use_sim_time:=true")
      ;;
    --no-use-sim-time)
      LAUNCH_ARGS+=("use_sim_time:=false")
      ;;
    --rviz)
      LAUNCH_ARGS+=("use_rviz:=true")
      ;;
    --no-rviz)
      LAUNCH_ARGS+=("use_rviz:=false")
      ;;
    --publish-map-to-odom-tf)
      LAUNCH_ARGS+=("publish_map_to_odom_tf:=true")
      ;;
    --no-publish-map-to-odom-tf)
      LAUNCH_ARGS+=("publish_map_to_odom_tf:=false")
      ;;
    --lio-sam-package=*)
      LAUNCH_ARGS+=("lio_sam_package:=${1#*=}")
      ;;
    --lio-sam-package)
      shift
      LAUNCH_ARGS+=("lio_sam_package:=$(require_value --lio-sam-package "${1:-}")")
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

if [[ -n "${LEFT_POINTS_TOPIC}" || -n "${RIGHT_POINTS_TOPIC}" ]]; then
  input_topics=()
  [[ -n "${LEFT_POINTS_TOPIC}" ]] && input_topics+=("${LEFT_POINTS_TOPIC}")
  [[ -n "${RIGHT_POINTS_TOPIC}" ]] && input_topics+=("${RIGHT_POINTS_TOPIC}")
  LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
  if [[ -n "${LEFT_POINTS_TOPIC}" ]]; then
    LAUNCH_ARGS+=("reference_points_topic:=${LEFT_POINTS_TOPIC}")
  fi
fi

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  # --build指定時だけworkspace setupを実行し、通常起動では再buildを避ける。
  bash "${SETUP_RUNTIME_SCRIPT}"
fi

if [[ ! -f "${AITRAN_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing aitran/ros2_ws/install/setup.bash. Run bash aitran/scripts/install/setup.sh first." >&2
  exit 1
fi

source_workspace_environment

ros2 launch ai_ship_robot_slam lio_sam.launch.py "${LAUNCH_ARGS[@]}"
