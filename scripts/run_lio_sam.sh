#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_RUNTIME_SCRIPT="${WORKSPACE_ROOT}/install/setup.sh"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_UNDERLAY_SETUP="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"

usage() {
  cat <<'EOF'
Usage: bash scripts/run_lio_sam.sh [OPTIONS]

Options:
  --build                     Build/install required workspace profile before launch.
  --config PATH               Use a LIO-SAM params YAML file.
  --fusion-config PATH        Use a multi-LiDAR fusion params YAML file.
  --input-points CSV          Set optional fusion input CustomMsg topics as comma-separated list.
  --left-points TOPIC         Compatibility alias for adding / replacing left LiDAR CustomMsg input.
  --right-points TOPIC        Compatibility alias for adding / replacing right LiDAR CustomMsg input.
  --reference-points TOPIC    Set reference LiDAR CustomMsg topic.
  --reference-lidar-frame FRAME
                              Set reference LiDAR frame used for fusion output.
  --fused-points TOPIC        Set optional fusion PointCloud2 debug topic.
  --raw-imu TOPIC             Set raw 6-axis IMU topic for initial orientation estimation.
  --imu TOPIC                 Set LIO-SAM IMU topic. Default is /livox/imu.
  --imu-type TYPE             Set IMU mode: six_axis | nine_axis. Default is six_axis.
  --imu-acceleration-unit U   Set IMU acceleration unit: g | mps2. Default is g.
  --imu-acceleration-scale S  Additional IMU acceleration scale. Default is 1.0.
  --imu-frequency HZ          Set fallback IMU frequency. Default is 500.0.
  --expected-acceleration-norm N
                              Set initial 6-axis IMU acceleration norm.
  --acceleration-norm-tolerance N
                              Set initial 6-axis IMU acceleration norm tolerance.
  --imu-debug                 Enable converted IMU diagnostic logs.
  --no-imu-debug              Disable converted IMU diagnostic logs. Default.
  --deskew-mode MODE          Set deskew mode: imu_angular | odom_interpolation | off.
  --wait-for-imu-initialization
                              Wait for internal 6-axis IMU initialization. Default.
  --no-wait-for-imu-initialization
                              Do not wait for internal 6-axis IMU initialization.
  --use-imu-preintegration-initial-guess
                              Use /odometry/imu_incremental as map initial guess. Default.
  --no-use-imu-preintegration-initial-guess
                              Disable preintegration initial guess.
  --use-imu-translation-initial-guess
                              Use IMU preintegration translation as initial guess.
  --no-use-imu-translation-initial-guess
                              Ignore IMU preintegration translation. Default.
  --use-imu-rotation-initial-guess
                              Use IMU preintegration rotation as initial guess. Default.
  --no-use-imu-rotation-initial-guess
                              Ignore IMU preintegration rotation.
  --lio-custom TOPIC          Set corrected CustomMsg topic passed to UV-Lab LIO-SAM.
  --lio-points TOPIC          Compatibility alias for --lio-custom.
  --imu-initializer           Enable legacy external 6-axis IMU roll/pitch estimator.
  --no-imu-initializer        Disable legacy external estimator. Default.
  --fusion                    Enable optional multi-LiDAR fusion node.
  --no-fusion                 Disable optional multi-LiDAR fusion node. Default.
  --adapter                   Enable legacy PointCloud2 adapter for debugging.
  --no-adapter                Disable legacy PointCloud2 adapter. Default.
  --derived-ring-count N      Set pseudo ring count when raw cloud has no ring/line field.
  --min-vertical-angle DEG    Set minimum vertical angle for pseudo ring derivation.
  --max-vertical-angle DEG    Set maximum vertical angle for pseudo ring derivation.
  --fusion-timestamp-scale S  Set integer point timestamp unit scale for fusion.
  --use-sim-time              Use ROS simulation time for Gazebo or rosbag playback.
  --no-use-sim-time           Use system time. Default.
  --rviz                      Enable RViz2.
  --no-rviz                   Disable RViz2.
  --rviz-config PATH          Use a workspace RViz config file.
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
    echo "Run bash install/install_third_party.sh && bash scripts/run_lio_sam.sh --build." >&2
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
    if ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash"; then
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
      LAUNCH_ARGS+=("use_fusion:=true")
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
      LAUNCH_ARGS+=("use_fusion:=true")
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
      ;;
    --points=*)
      points_topic="${1#*=}"
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${points_topic}")")
      LAUNCH_ARGS+=("reference_points_topic:=${points_topic}")
      LAUNCH_ARGS+=("lio_custom_topic:=${points_topic}")
      ;;
    --points)
      shift
      points_topic="$(require_value --points "${1:-}")"
      LAUNCH_ARGS+=("input_points_topics:=$(format_topic_list "${points_topic}")")
      LAUNCH_ARGS+=("reference_points_topic:=${points_topic}")
      LAUNCH_ARGS+=("lio_custom_topic:=${points_topic}")
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
      LAUNCH_ARGS+=("use_fusion:=true")
      LAUNCH_ARGS+=("reference_points_topic:=${1#*=}")
      ;;
    --reference-points)
      shift
      LAUNCH_ARGS+=("use_fusion:=true")
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
      LAUNCH_ARGS+=("use_fusion:=true")
      LAUNCH_ARGS+=("fused_points_topic:=${1#*=}")
      ;;
    --fused-points)
      shift
      LAUNCH_ARGS+=("use_fusion:=true")
      LAUNCH_ARGS+=("fused_points_topic:=$(require_value --fused-points "${1:-}")")
      ;;
    --imu=*)
      LAUNCH_ARGS+=("imu_topic:=${1#*=}")
      ;;
    --imu)
      shift
      LAUNCH_ARGS+=("imu_topic:=$(require_value --imu "${1:-}")")
      ;;
    --imu-type=*)
      LAUNCH_ARGS+=("imu_type:=${1#*=}")
      ;;
    --imu-type)
      shift
      LAUNCH_ARGS+=("imu_type:=$(require_value --imu-type "${1:-}")")
      ;;
    --imu-acceleration-unit=*)
      LAUNCH_ARGS+=("imu_acceleration_unit:=${1#*=}")
      ;;
    --imu-acceleration-unit)
      shift
      LAUNCH_ARGS+=("imu_acceleration_unit:=$(require_value --imu-acceleration-unit "${1:-}")")
      ;;
    --imu-acceleration-scale=*)
      LAUNCH_ARGS+=("imu_acceleration_scale:=${1#*=}")
      ;;
    --imu-acceleration-scale)
      shift
      LAUNCH_ARGS+=("imu_acceleration_scale:=$(require_value --imu-acceleration-scale "${1:-}")")
      ;;
    --imu-frequency=*)
      LAUNCH_ARGS+=("imu_frequency:=${1#*=}")
      ;;
    --imu-frequency)
      shift
      LAUNCH_ARGS+=("imu_frequency:=$(require_value --imu-frequency "${1:-}")")
      ;;
    --expected-acceleration-norm=*)
      LAUNCH_ARGS+=("expected_acceleration_norm:=${1#*=}")
      ;;
    --expected-acceleration-norm)
      shift
      LAUNCH_ARGS+=("expected_acceleration_norm:=$(require_value --expected-acceleration-norm "${1:-}")")
      ;;
    --acceleration-norm-tolerance=*)
      LAUNCH_ARGS+=("acceleration_norm_tolerance:=${1#*=}")
      ;;
    --acceleration-norm-tolerance)
      shift
      LAUNCH_ARGS+=("acceleration_norm_tolerance:=$(require_value --acceleration-norm-tolerance "${1:-}")")
      ;;
    --imu-debug)
      LAUNCH_ARGS+=("imu_debug:=true")
      ;;
    --no-imu-debug)
      LAUNCH_ARGS+=("imu_debug:=false")
      ;;
    --deskew-mode=*)
      LAUNCH_ARGS+=("deskew_mode:=${1#*=}")
      ;;
    --deskew-mode)
      shift
      LAUNCH_ARGS+=("deskew_mode:=$(require_value --deskew-mode "${1:-}")")
      ;;
    --wait-for-imu-initialization)
      LAUNCH_ARGS+=("wait_for_imu_initialization:=true")
      ;;
    --no-wait-for-imu-initialization)
      LAUNCH_ARGS+=("wait_for_imu_initialization:=false")
      ;;
    --use-imu-preintegration-initial-guess)
      LAUNCH_ARGS+=("use_imu_preintegration_initial_guess:=true")
      ;;
    --no-use-imu-preintegration-initial-guess)
      LAUNCH_ARGS+=("use_imu_preintegration_initial_guess:=false")
      ;;
    --use-imu-translation-initial-guess)
      LAUNCH_ARGS+=("use_imu_translation_initial_guess:=true")
      ;;
    --no-use-imu-translation-initial-guess)
      LAUNCH_ARGS+=("use_imu_translation_initial_guess:=false")
      ;;
    --use-imu-rotation-initial-guess)
      LAUNCH_ARGS+=("use_imu_rotation_initial_guess:=true")
      ;;
    --no-use-imu-rotation-initial-guess)
      LAUNCH_ARGS+=("use_imu_rotation_initial_guess:=false")
      ;;
    --raw-imu=*)
      LAUNCH_ARGS+=("raw_imu_topic:=${1#*=}")
      ;;
    --raw-imu)
      shift
      LAUNCH_ARGS+=("raw_imu_topic:=$(require_value --raw-imu "${1:-}")")
      ;;
    --imu-initializer)
      LAUNCH_ARGS+=("use_imu_orientation_initializer:=true")
      ;;
    --no-imu-initializer)
      LAUNCH_ARGS+=("use_imu_orientation_initializer:=false")
      ;;
    --lio-points=*)
      LAUNCH_ARGS+=("lio_custom_topic:=${1#*=}")
      ;;
    --lio-points)
      shift
      LAUNCH_ARGS+=("lio_custom_topic:=$(require_value --lio-points "${1:-}")")
      ;;
    --lio-custom=*)
      LAUNCH_ARGS+=("lio_custom_topic:=${1#*=}")
      ;;
    --lio-custom)
      shift
      LAUNCH_ARGS+=("lio_custom_topic:=$(require_value --lio-custom "${1:-}")")
      ;;
    --adapter)
      LAUNCH_ARGS+=("use_adapter:=true")
      ;;
    --no-adapter)
      LAUNCH_ARGS+=("use_adapter:=false")
      ;;
    --fusion)
      LAUNCH_ARGS+=("use_fusion:=true")
      ;;
    --no-fusion)
      LAUNCH_ARGS+=("use_fusion:=false")
      ;;
    --map-to-odom-z|--map-to-odom-z=*)
      echo "--map-to-odom-z has been removed. Align world/odom later with an external static TF." >&2
      exit 2
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
    --rviz-config=*)
      LAUNCH_ARGS+=("rviz_config:=${1#*=}")
      ;;
    --rviz-config)
      shift
      LAUNCH_ARGS+=("rviz_config:=$(require_value --rviz-config "${1:-}")")
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
  LAUNCH_ARGS+=("use_fusion:=true")
  if [[ -n "${LEFT_POINTS_TOPIC}" ]]; then
    LAUNCH_ARGS+=("reference_points_topic:=${LEFT_POINTS_TOPIC}")
  fi
fi

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  # --build指定時だけworkspace setupを実行し、通常起動では再buildを避ける。
  bash "${SETUP_RUNTIME_SCRIPT}"
fi

if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing ros2_ws/install/setup.bash. Run bash install/setup.sh first." >&2
  exit 1
fi

source_workspace_environment

ros2 launch ai_ship_robot_slam lio_sam.launch.py "${LAUNCH_ARGS[@]}"
