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
DEFAULT_LIDAR_TOPICS=("/livox/lidar")
DEFAULT_IMU_TOPICS=("/livox/imu")
SIM_RECORD_LIDAR_TOPICS=("/left_lidar/custom" "/right_lidar/custom" "/livox/lidar")
SIM_RECORD_IMU_TOPICS=("/left_lidar/imu" "/right_lidar/imu" "/livox/imu")
ROSBAG_PID=""
SLAM_PID=""
ROSBAG_ROOT="${WORKSPACE_ROOT}/rosbag2"

usage() {
  cat <<'EOF'
Usage: bash aitran/scripts/app/run_slam.sh [OPTIONS]

Options:
  --sim              Launch Gazebo simulation and LIO-SAM together.
  --record-bag       Record LiDAR CustomMsg, IMU, /clock, /tf, and /tf_static during SLAM execution.
  --bag-output PATH  Set rosbag output directory or prefix.
  --bag-topics CSV   Add extra record topics as comma-separated list.
  --imu-topics CSV   Override recorded IMU topics as comma-separated list.
  --bag-play PATH    Play a recorded rosbag and run LIO-SAM without Gazebo.
  --bag-play-rate N  Set rosbag playback rate. Default: 1.0
  --bag-start-offset SEC
                     Start rosbag playback after the given offset.
  --bag-loop         Loop rosbag playback.
  --backend lio-sam  Accepted for compatibility.
  --lio-sam          Accepted for compatibility.
  -h, --help         Show this help.

Examples:
  bash aitran/scripts/app/run_slam.sh --no-rviz
  bash aitran/scripts/app/run_slam.sh --sim --lite --no-gui
  bash aitran/scripts/app/run_slam.sh --points /livox/lidar --raw-imu /livox/imu --imu /livox/imu_oriented
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

append_unique_topics() {
  local array_name="$1"
  shift
  local -n target_array="${array_name}"
  local candidate=""
  local existing=""
  local found=0

  for candidate in "$@"; do
    [[ -n "${candidate}" ]] || continue
    found=0
    for existing in "${target_array[@]}"; do
      if [[ "${existing}" == "${candidate}" ]]; then
        found=1
        break
      fi
    done
    if [[ "${found}" -eq 0 ]]; then
      target_array+=("${candidate}")
    fi
  done
}

build_passthrough_args() {
  local filtered_args=()
  local arg=""
  local skip_next=0

  # wrapper専用引数はここで除去し、run_lio_sam.shにはLIO-SAM固有オプションだけを渡す。
  for arg in "${FORWARD_ARGS[@]}"; do
    if [[ "${skip_next}" -eq 1 ]]; then
      skip_next=0
      continue
    fi
    case "${arg}" in
      --record-bag|--bag-topics=*|--imu-topics=*|--bag-output=*)
        ;;
      --bag-topics|--imu-topics|--bag-output)
        skip_next=1
        ;;
      *)
        filtered_args+=("${arg}")
        ;;
    esac
  done

  if [[ "${#filtered_args[@]}" -gt 0 ]]; then
    printf '%s\n' "${filtered_args[@]}"
  fi
}

get_launch_arg_value() {
  local prefix="$1"
  shift
  local arg=""
  local value=""

  for arg in "$@"; do
    if [[ "${arg}" == "${prefix}:="* ]]; then
      value="${arg#${prefix}:=}"
    fi
  done

  printf '%s' "${value}"
}

parse_launch_topic_list() {
  local serialized="$1"

  serialized="${serialized#[}"
  serialized="${serialized%]}"
  serialized="${serialized//\'/}"
  parse_csv_topics "${serialized}"
}

start_rosbag_record() {
  local output_path="$1"
  local use_sim_time="$2"
  shift 2
  local topics=("$@")
  local record_cmd=(ros2 bag record --include-hidden-topics)

  mkdir -p "${ROSBAG_ROOT}"
  if [[ -n "${output_path}" ]]; then
    record_cmd+=(--output "${output_path}")
  fi
  if [[ "${use_sim_time}" == "true" ]]; then
    record_cmd+=(--use-sim-time)
  fi

  record_cmd+=("${topics[@]}")
  echo "Rosbag output: ${output_path}" >&2
  echo "Recording rosbag topics: ${topics[*]}" >&2
  "${record_cmd[@]}" &
  ROSBAG_PID=$!
}

default_bag_output() {
  local prefix="$1"

  # 用途別prefixを付け、収録元を識別しやすい保存先名にする。
  printf '%s/%s_%(%Y%m%d_%H%M%S)T' "${ROSBAG_ROOT}" "${prefix}" -1
}

cleanup_background_processes() {
  # bag再生終了時にSLAMも止め、逆に終了時はrecord/play側も確実に片付ける。
  if [[ -n "${ROSBAG_PID}" ]] && kill -0 "${ROSBAG_PID}" 2>/dev/null; then
    kill -INT "${ROSBAG_PID}" 2>/dev/null || kill -TERM "${ROSBAG_PID}" 2>/dev/null || true
    wait "${ROSBAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SLAM_PID}" ]] && kill -0 "${SLAM_PID}" 2>/dev/null; then
    kill -INT "${SLAM_PID}" 2>/dev/null || kill -TERM "${SLAM_PID}" 2>/dev/null || true
    wait "${SLAM_PID}" 2>/dev/null || true
  fi
}

run_recorded_lio_sam() {
  local bag_output="$1"
  shift
  local topics=("$@")
  local slam_args=()

  start_rosbag_record "${bag_output}" false "${topics[@]}"
  mapfile -t slam_args < <(build_passthrough_args)
  bash "${SCRIPT_DIR}/run_lio_sam.sh" "${slam_args[@]}"
}

run_bag_play_lio_sam() {
  local bag_path="$1"
  local bag_rate="$2"
  local bag_offset="$3"
  local bag_loop="$4"
  local play_cmd=(ros2 bag play "${bag_path}" --clock --rate "${bag_rate}" --start-offset "${bag_offset}")
  local slam_args=()
  local play_topics=()
  local derived_input_topics=""
  local derived_imu_topic=""
  local derived_raw_imu_topic=""

  if [[ "${bag_loop}" == "true" ]]; then
    play_cmd+=(--loop)
  fi

  for ((i = 0; i < ${#FORWARD_ARGS[@]}; i++)); do
    case "${FORWARD_ARGS[i]}" in
      --input-points=*)
        derived_input_topics="${FORWARD_ARGS[i]#*=}"
        ;;
      --input-points)
        ((i++))
        derived_input_topics="${FORWARD_ARGS[i]}"
        ;;
      --points=*)
        derived_input_topics="${FORWARD_ARGS[i]#*=}"
        ;;
      --points)
        ((i++))
        derived_input_topics="${FORWARD_ARGS[i]}"
        ;;
      --imu=*)
        derived_imu_topic="${FORWARD_ARGS[i]#*=}"
        ;;
      --imu)
        ((i++))
        derived_imu_topic="${FORWARD_ARGS[i]}"
        ;;
      --raw-imu=*)
        derived_raw_imu_topic="${FORWARD_ARGS[i]#*=}"
        ;;
      --raw-imu)
        ((i++))
        derived_raw_imu_topic="${FORWARD_ARGS[i]}"
        ;;
    esac
  done

  if [[ -n "${derived_input_topics}" ]]; then
    mapfile -t play_topics < <(parse_csv_topics "${derived_input_topics}")
  else
    play_topics=("${DEFAULT_LIDAR_TOPICS[@]}")
  fi

  if [[ -n "${derived_raw_imu_topic}" ]]; then
    append_unique_topics play_topics "${derived_raw_imu_topic}"
  else
    append_unique_topics play_topics "${DEFAULT_IMU_TOPICS[@]}"
  fi
  append_unique_topics play_topics "/tf_static"
  play_cmd+=(--topics "${play_topics[@]}")

  mapfile -t slam_args < <(build_passthrough_args)
  bash "${SCRIPT_DIR}/run_lio_sam.sh" --use-sim-time "${slam_args[@]}" &
  SLAM_PID=$!
  sleep 2
  "${play_cmd[@]}" &
  ROSBAG_PID=$!
  wait "${ROSBAG_PID}"
}

run_sim_lio_sam() {
  local build_workspace=false
  local lite_mode=false
  local robot_name_set=false
  local launch_args=()
  local left_points_topic=""
  local right_points_topic=""
  local record_bag=false
  local bag_output=""
  local bag_topics=()
  local lidar_topics=("${SIM_RECORD_LIDAR_TOPICS[@]}")
  local imu_topics=("${SIM_RECORD_IMU_TOPICS[@]}")

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
      --fusion-config=*)
        launch_args+=("fusion_config:=${1#*=}")
        ;;
      --fusion-config)
        shift
        launch_args+=("fusion_config:=$(require_value --fusion-config "${1:-}")")
        ;;
      --input-points=*)
        mapfile -t input_topics < <(parse_csv_topics "${1#*=}")
        if [[ "${#input_topics[@]}" -eq 0 ]]; then
          echo "--input-points requires at least one topic." >&2
          exit 2
        fi
        launch_args+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
        ;;
      --input-points)
        shift
        input_points_value="$(require_value --input-points "${1:-}")"
        mapfile -t input_topics < <(parse_csv_topics "${input_points_value}")
        if [[ "${#input_topics[@]}" -eq 0 ]]; then
          echo "--input-points requires at least one topic." >&2
          exit 2
        fi
        launch_args+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
        ;;
      --points=*)
        points_topic="${1#*=}"
        launch_args+=("input_points_topics:=$(format_topic_list "${points_topic}")")
        launch_args+=("reference_points_topic:=${points_topic}")
        launch_args+=("lio_custom_topic:=${points_topic}")
        ;;
      --points)
        shift
        points_topic="$(require_value --points "${1:-}")"
        launch_args+=("input_points_topics:=$(format_topic_list "${points_topic}")")
        launch_args+=("reference_points_topic:=${points_topic}")
        launch_args+=("lio_custom_topic:=${points_topic}")
        ;;
      --left-points=*)
        left_points_topic="${1#*=}"
        ;;
      --left-points)
        shift
        left_points_topic="$(require_value --left-points "${1:-}")"
        ;;
      --right-points=*)
        right_points_topic="${1#*=}"
        ;;
      --right-points)
        shift
        right_points_topic="$(require_value --right-points "${1:-}")"
        ;;
      --reference-points=*)
        launch_args+=("reference_points_topic:=${1#*=}")
        ;;
      --reference-points)
        shift
        launch_args+=("reference_points_topic:=$(require_value --reference-points "${1:-}")")
        ;;
      --reference-lidar-frame=*)
        launch_args+=("reference_lidar_frame:=${1#*=}")
        ;;
      --reference-lidar-frame)
        shift
        launch_args+=("reference_lidar_frame:=$(require_value --reference-lidar-frame "${1:-}")")
        ;;
      --fused-points=*)
        launch_args+=("fused_points_topic:=${1#*=}")
        ;;
      --fused-points)
        shift
        launch_args+=("fused_points_topic:=$(require_value --fused-points "${1:-}")")
        ;;
      --imu=*)
        launch_args+=("imu_topic:=${1#*=}")
        ;;
      --imu)
        shift
        launch_args+=("imu_topic:=$(require_value --imu "${1:-}")")
        ;;
      --raw-imu=*)
        launch_args+=("raw_imu_topic:=${1#*=}")
        ;;
      --raw-imu)
        shift
        launch_args+=("raw_imu_topic:=$(require_value --raw-imu "${1:-}")")
        ;;
      --imu-initializer)
        launch_args+=("use_imu_orientation_initializer:=true")
        ;;
      --no-imu-initializer)
        launch_args+=("use_imu_orientation_initializer:=false")
        ;;
      --record-bag)
        record_bag=true
        ;;
      --bag-output=*)
        bag_output="${1#*=}"
        ;;
      --bag-output)
        shift
        bag_output="$(require_value --bag-output "${1:-}")"
        ;;
      --bag-topics=*)
        mapfile -t bag_topics < <(parse_csv_topics "${1#*=}")
        ;;
      --bag-topics)
        shift
        mapfile -t bag_topics < <(parse_csv_topics "$(require_value --bag-topics "${1:-}")")
        ;;
      --lidar-topics=*)
        mapfile -t lidar_topics < <(parse_csv_topics "${1#*=}")
        ;;
      --lidar-topics)
        shift
        mapfile -t lidar_topics < <(parse_csv_topics "$(require_value --lidar-topics "${1:-}")")
        ;;
      --imu-topics=*)
        mapfile -t imu_topics < <(parse_csv_topics "${1#*=}")
        ;;
      --imu-topics)
        shift
        mapfile -t imu_topics < <(parse_csv_topics "$(require_value --imu-topics "${1:-}")")
        ;;
      --lio-points=*)
        launch_args+=("lio_custom_topic:=${1#*=}")
        ;;
      --lio-points)
        shift
        launch_args+=("lio_custom_topic:=$(require_value --lio-points "${1:-}")")
        ;;
      --lio-custom=*)
        launch_args+=("lio_custom_topic:=${1#*=}")
        ;;
      --lio-custom)
        shift
        launch_args+=("lio_custom_topic:=$(require_value --lio-custom "${1:-}")")
        ;;
      --adapter)
        launch_args+=("use_adapter:=true")
        ;;
      --no-adapter)
        launch_args+=("use_adapter:=false")
        ;;
      --fusion)
        launch_args+=("use_fusion:=true")
        ;;
      --no-fusion)
        launch_args+=("use_fusion:=false")
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
      --fusion-timestamp-scale=*)
        launch_args+=("fusion_timestamp_unit_scale:=${1#*=}")
        ;;
      --fusion-timestamp-scale)
        shift
        launch_args+=("fusion_timestamp_unit_scale:=$(require_value --fusion-timestamp-scale "${1:-}")")
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

  if [[ -n "${left_points_topic}" || -n "${right_points_topic}" ]]; then
    input_topics=()
    [[ -n "${left_points_topic}" ]] && input_topics+=("${left_points_topic}")
    [[ -n "${right_points_topic}" ]] && input_topics+=("${right_points_topic}")
    launch_args+=("input_points_topics:=$(format_topic_list "${input_topics[@]}")")
    if [[ -n "${left_points_topic}" ]]; then
      launch_args+=("reference_points_topic:=${left_points_topic}")
    fi
  fi

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
  if [[ "${record_bag}" == "true" ]]; then
    if [[ -z "${bag_output}" ]]; then
      bag_output="$(default_bag_output sim_slam)"
    fi
    derived_input_topics="$(get_launch_arg_value input_points_topics "${launch_args[@]}")"
    derived_raw_imu_topic="$(get_launch_arg_value raw_imu_topic "${launch_args[@]}")"
    record_topics=()
    if [[ -n "${derived_input_topics}" ]]; then
      mapfile -t lidar_topics < <(parse_launch_topic_list "${derived_input_topics}")
    fi
    if [[ -n "${derived_raw_imu_topic}" && "${#imu_topics[@]}" -eq 3 && "${imu_topics[2]}" == "/livox/imu" ]]; then
      imu_topics=("${derived_raw_imu_topic}")
    fi
    append_unique_topics record_topics "${lidar_topics[@]}"
    append_unique_topics record_topics "${imu_topics[@]}"
    append_unique_topics record_topics "${bag_topics[@]}"
    append_unique_topics record_topics "/clock" "/tf" "/tf_static"
  fi
  if [[ "${record_bag}" == "true" ]]; then
    ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}" &
    start_rosbag_record "${bag_output}" true "${record_topics[@]}"
    wait
    return $?
  fi
  ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}"
}

RECORD_BAG=false
BAG_OUTPUT=""
BAG_TOPICS=()
IMU_TOPICS=("${DEFAULT_IMU_TOPICS[@]}")
BAG_PLAY=""
BAG_PLAY_RATE="1.0"
BAG_START_OFFSET="0"
BAG_LOOP=false

trap cleanup_background_processes EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --sim)
      SIM_MODE=true
      ;;
    --record-bag)
      RECORD_BAG=true
      FORWARD_ARGS+=("$1")
      ;;
    --bag-output=*)
      BAG_OUTPUT="${1#*=}"
      FORWARD_ARGS+=("$1")
      ;;
    --bag-output)
      shift
      BAG_OUTPUT="$(require_value --bag-output "${1:-}")"
      FORWARD_ARGS+=("--bag-output" "${BAG_OUTPUT}")
      ;;
    --bag-topics=*)
      mapfile -t BAG_TOPICS < <(parse_csv_topics "${1#*=}")
      FORWARD_ARGS+=("$1")
      ;;
    --bag-topics)
      shift
      bag_topics_value="$(require_value --bag-topics "${1:-}")"
      mapfile -t BAG_TOPICS < <(parse_csv_topics "${bag_topics_value}")
      FORWARD_ARGS+=("--bag-topics" "${bag_topics_value}")
      ;;
    --imu-topics=*)
      mapfile -t IMU_TOPICS < <(parse_csv_topics "${1#*=}")
      FORWARD_ARGS+=("$1")
      ;;
    --imu-topics)
      shift
      imu_topics_value="$(require_value --imu-topics "${1:-}")"
      mapfile -t IMU_TOPICS < <(parse_csv_topics "${imu_topics_value}")
      FORWARD_ARGS+=("--imu-topics" "${imu_topics_value}")
      ;;
    --bag-play=*)
      BAG_PLAY="${1#*=}"
      ;;
    --bag-play)
      shift
      BAG_PLAY="$(require_value --bag-play "${1:-}")"
      ;;
    --bag-play-rate=*)
      BAG_PLAY_RATE="${1#*=}"
      ;;
    --bag-play-rate)
      shift
      BAG_PLAY_RATE="$(require_value --bag-play-rate "${1:-}")"
      ;;
    --bag-start-offset=*)
      BAG_START_OFFSET="${1#*=}"
      ;;
    --bag-start-offset)
      shift
      BAG_START_OFFSET="$(require_value --bag-start-offset "${1:-}")"
      ;;
    --bag-loop)
      BAG_LOOP=true
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
  if [[ -n "${BAG_PLAY}" ]]; then
    echo "--sim and --bag-play cannot be used together." >&2
    exit 2
  fi
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

if [[ -n "${BAG_PLAY}" && "${RECORD_BAG}" == "true" ]]; then
  echo "--bag-play and --record-bag cannot be used together." >&2
  exit 2
fi

if [[ -n "${BAG_PLAY}" ]]; then
  for arg in "${FORWARD_ARGS[@]}"; do
    if [[ "${arg}" == "--no-use-sim-time" ]]; then
      echo "--bag-play always runs with use_sim_time=true; --no-use-sim-time cannot be used." >&2
      exit 2
    fi
  done
  source_sim_slam_environment
  run_bag_play_lio_sam "${BAG_PLAY}" "${BAG_PLAY_RATE}" "${BAG_START_OFFSET}" "${BAG_LOOP}"
  exit $?
fi

if [[ "${RECORD_BAG}" == "true" ]]; then
  source_sim_slam_environment
  if [[ -z "${BAG_OUTPUT}" ]]; then
    BAG_OUTPUT="$(default_bag_output lio_sam)"
  fi
  RECORD_TOPICS=()
  DERIVED_INPUT_TOPICS=""
  DERIVED_IMU_TOPIC=""
  DERIVED_RAW_IMU_TOPIC=""
  for ((i = 0; i < ${#FORWARD_ARGS[@]}; i++)); do
    case "${FORWARD_ARGS[i]}" in
      --input-points=*)
        DERIVED_INPUT_TOPICS="${FORWARD_ARGS[i]#*=}"
        ;;
      --input-points)
        ((i++))
        DERIVED_INPUT_TOPICS="${FORWARD_ARGS[i]}"
        ;;
      --points=*)
        DERIVED_INPUT_TOPICS="${FORWARD_ARGS[i]#*=}"
        ;;
      --points)
        ((i++))
        DERIVED_INPUT_TOPICS="${FORWARD_ARGS[i]}"
        ;;
      --imu=*)
        DERIVED_IMU_TOPIC="${FORWARD_ARGS[i]#*=}"
        ;;
      --imu)
        ((i++))
        DERIVED_IMU_TOPIC="${FORWARD_ARGS[i]}"
        ;;
      --raw-imu=*)
        DERIVED_RAW_IMU_TOPIC="${FORWARD_ARGS[i]#*=}"
        ;;
      --raw-imu)
        ((i++))
        DERIVED_RAW_IMU_TOPIC="${FORWARD_ARGS[i]}"
        ;;
    esac
  done
  if [[ -n "${DERIVED_INPUT_TOPICS}" ]]; then
    mapfile -t LIDAR_TOPICS < <(parse_csv_topics "${DERIVED_INPUT_TOPICS}")
  else
    LIDAR_TOPICS=("${DEFAULT_LIDAR_TOPICS[@]}")
  fi
  if [[ -n "${DERIVED_RAW_IMU_TOPIC}" && "${#IMU_TOPICS[@]}" -eq 1 && "${IMU_TOPICS[0]}" == "/livox/imu" ]]; then
    IMU_TOPICS=("${DERIVED_RAW_IMU_TOPIC}")
  fi
  append_unique_topics RECORD_TOPICS "${LIDAR_TOPICS[@]}"
  append_unique_topics RECORD_TOPICS "${IMU_TOPICS[@]}"
  append_unique_topics RECORD_TOPICS "${BAG_TOPICS[@]}"
  append_unique_topics RECORD_TOPICS "/clock" "/tf" "/tf_static"
  run_recorded_lio_sam "${BAG_OUTPUT}" "${RECORD_TOPICS[@]}"
  exit $?
fi

# 通常時の汎用入口はLIO-SAM単体起動scriptへの互換ラッパーとして扱う。
if [[ "${#IMU_TOPICS[@]}" -gt 0 || "${#BAG_TOPICS[@]}" -gt 0 || -n "${BAG_OUTPUT}" ]]; then
  mapfile -t PASSTHROUGH_ARGS < <(build_passthrough_args)
  exec bash "${SCRIPT_DIR}/run_lio_sam.sh" "${PASSTHROUGH_ARGS[@]}"
fi

exec bash "${SCRIPT_DIR}/run_lio_sam.sh" "${FORWARD_ARGS[@]}"
