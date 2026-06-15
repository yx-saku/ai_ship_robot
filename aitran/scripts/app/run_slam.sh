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
DEFAULT_IMU_TOPICS=("/left_lidar/imu")
ROSBAG_PID=""
ROSBAG_PLAY_PID=""
SLAM_PID=""
ROSBAG_ROOT="${WORKSPACE_ROOT}/rosbag2"
AUTO_STOP_AFTER_BAG_PLAY_SECONDS="${AUTO_STOP_AFTER_BAG_PLAY_SECONDS:-30}"
PROCESS_STOP_GRACE_SECONDS="15"
PROCESS_STOP_TERM_SECONDS="5"
ROSBAG_STOP_GRACE_SECONDS="60"

usage() {
  cat <<'EOF'
Usage: bash aitran/scripts/app/run_slam.sh [OPTIONS]

Options:
  --sim              Launch Gazebo simulation and LIO-SAM together.
  --record-bag       Record all topics during SLAM execution.
  --bag-output PATH  Set rosbag output directory or prefix.
  --bag-topics CSV   Record only the given comma-separated topics. Default records all topics.
  --bag-play [PATH]  Play a recorded rosbag and run LIO-SAM without Gazebo.
                     If PATH is omitted, the latest rosbag2/sim_* bag is used.
  --bag-play-rate N  Set rosbag playback rate. Default: 1.0
  --bag-start-delay SEC
                     Delay rosbag playback start by the given seconds.
  --bag-start-offset SEC
                      Start rosbag playback after the given offset.
  --auto-stop-after-bag-play SEC
                      Seconds to keep SLAM running after playback finishes. Default: 30.
  --no-auto-exit     Keep SLAM and recording running after rosbag playback finishes.
  --rviz-config PATH Use a workspace RViz config file for LIO-SAM.
  --deskew-mode MODE Set LIO-SAM deskew mode: imu_angular | odom_interpolation | off.
  --force-zero-offset-time
                     Force simulated Livox point offset_time to zero. Default for --sim.
  --no-force-zero-offset-time
                     Preserve simulated Livox point offset_time.
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
      --record-bag|--bag-topics=*|--bag-output=*)
        ;;
      --bag-topics|--bag-output)
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

  # topic無指定時は明示的に全topic記録へ切り替え、入力topicの後追い発見もrecord対象にする。
  if [[ "${#topics[@]}" -eq 0 ]]; then
    record_cmd+=(--all)
  else
    record_cmd+=("${topics[@]}")
  fi
  echo "Rosbag output: ${output_path}" >&2
  if [[ "${#topics[@]}" -eq 0 ]]; then
    echo "Recording rosbag topics: all topics" >&2
  else
    echo "Recording rosbag topics: ${topics[*]}" >&2
  fi
  "${record_cmd[@]}" &
  ROSBAG_PID=$!
}

default_bag_output() {
  # run_slam経由の収録はbackendや入力形態に関係なくslam prefixへ統一する。
  printf '%s/slam_%(%Y%m%d_%H%M%S)T' "${ROSBAG_ROOT}" -1
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

  # metadata.yamlを持つbagだけを候補にし、作成途中や壊れたディレクトリを再生対象から外す。
  if [[ -z "${newest_dir}" ]]; then
    echo "No rosbag found for prefix '${prefix}_' in ${ROSBAG_ROOT}." >&2
    exit 1
  fi

  printf '%s' "${newest_dir}"
}

collect_child_pids() {
  local parent_pid="$1"
  local child_pid=""

  # launch wrapperやrosbag配下の子プロセスも停止対象にし、終了後の残存プロセスを防ぐ。
  while IFS= read -r child_pid; do
    [[ -n "${child_pid}" ]] || continue
    collect_child_pids "${child_pid}"
    printf '%s\n' "${child_pid}"
  done < <(pgrep -P "${parent_pid}" 2>/dev/null || true)
}

signal_process_tree() {
  local signal_name="$1"
  local root_pid="$2"
  local child_pids=()
  local child_pid=""

  mapfile -t child_pids < <(collect_child_pids "${root_pid}")
  for child_pid in "${child_pids[@]}"; do
    kill "-${signal_name}" "${child_pid}" 2>/dev/null || true
  done
  kill "-${signal_name}" "${root_pid}" 2>/dev/null || true
}

stop_background_process() {
  local pid="$1"
  local label="$2"
  local grace_seconds="${3:-${PROCESS_STOP_GRACE_SECONDS}}"
  local term_seconds="${4:-${PROCESS_STOP_TERM_SECONDS}}"
  local stopper_pid=""
  local process_status=0

  [[ -n "${pid}" ]] || return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return 0
  fi

  echo "Stopping ${label}..." >&2
  signal_process_tree INT "${pid}"
  (
    sleep "${grace_seconds}"
    if kill -0 "${pid}" 2>/dev/null; then
      echo "${label} did not stop after ${grace_seconds}s; sending TERM." >&2
      signal_process_tree TERM "${pid}"
      sleep "${term_seconds}"
      if kill -0 "${pid}" 2>/dev/null; then
        echo "${label} did not stop after TERM; sending KILL." >&2
        signal_process_tree KILL "${pid}"
      fi
    fi
  ) &
  stopper_pid=$!

  # 子プロセスの停止が遅い場合でもtimer側で段階停止し、cleanupのwaitが無制限に残らないようにする。
  set +e
  wait "${pid}" 2>/dev/null
  process_status=$?
  kill "${stopper_pid}" 2>/dev/null || true
  wait "${stopper_pid}" 2>/dev/null || true
  set -e
  return "${process_status}"
}

cleanup_background_processes() {
  # bag再生終了時にSLAMも止め、逆に終了時はrecord/play側も確実に片付ける。
  stop_background_process "${ROSBAG_PID}" "rosbag recorder" "${ROSBAG_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  # bag再生プロセスはrecordとは別PIDで持ち、同時利用時も両方を確実に停止する。
  stop_background_process "${ROSBAG_PLAY_PID}" "rosbag player" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  stop_background_process "${SLAM_PID}" "SLAM" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
}

ensure_process_started() {
  local pid="$1"
  local process_name="$2"

  # 起動直後に落ちた子プロセスはここで検知し、後続のrecord/playを止めて原因を前面に出す。
  if kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  local process_status=0

  if wait "${pid}"; then
    process_status=0
  else
    process_status=$?
  fi
  echo "${process_name} exited before startup completed." >&2
  exit "${process_status}"
}

run_recorded_lio_sam() {
  local bag_output="$1"
  local slam_args=()

  # 単体SLAM収録では入力推定に依存せず、観測できるtopicを全て保存する。
  start_rosbag_record "${bag_output}" false
  mapfile -t slam_args < <(build_passthrough_args)
  bash "${SCRIPT_DIR}/run_lio_sam.sh" "${slam_args[@]}"
}

run_bag_play_lio_sam() {
  local bag_path="$1"
  local bag_rate="$2"
  local bag_offset="$3"
  local bag_start_delay="$4"
  local auto_exit="$5"
  local record_bag="$6"
  local bag_output="$7"
  shift 7
  local record_topics=("$@")
  local play_cmd=(ros2 bag play "${bag_path}" --clock --rate "${bag_rate}" --start-offset "${bag_offset}")
  local slam_args=()
  local play_topics=()
  local derived_input_topics=""
  local derived_imu_topic=""
  local derived_raw_imu_topic=""

  for ((i = 0; i < ${#FORWARD_ARGS[@]}; i++)); do
    case "${FORWARD_ARGS[i]}" in
      --input-points=*)
        derived_input_topics="${FORWARD_ARGS[i]#*=}"
        ;;
      --input-points)
        i=$((i + 1))
        derived_input_topics="${FORWARD_ARGS[i]}"
        ;;
      --points=*)
        derived_input_topics="${FORWARD_ARGS[i]#*=}"
        ;;
      --points)
        i=$((i + 1))
        derived_input_topics="${FORWARD_ARGS[i]}"
        ;;
      --imu=*)
        derived_imu_topic="${FORWARD_ARGS[i]#*=}"
        ;;
      --imu)
        i=$((i + 1))
        derived_imu_topic="${FORWARD_ARGS[i]}"
        ;;
      --raw-imu=*)
        derived_raw_imu_topic="${FORWARD_ARGS[i]#*=}"
        ;;
      --raw-imu)
        i=$((i + 1))
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
  bash "${SCRIPT_DIR}/run_lio_sam.sh" \
    --use-sim-time \
    --points /livox/lidar \
    --raw-imu /left_lidar/imu \
    --imu /left_lidar/imu \
    --no-imu-initializer \
    --imu-acceleration-unit mps2 \
    --expected-acceleration-norm 9.80511 \
    --acceleration-norm-tolerance 3.5 \
    --deskew-mode off \
    --imu-frequency 200.0 \
    "${slam_args[@]}" &
  SLAM_PID=$!
  sleep 2
  ensure_process_started "${SLAM_PID}" "LIO-SAM"
  # bag再生前にrecordを起動し、再生開始直後のtopicも取りこぼしにくくする。
  if [[ "${record_bag}" == "true" ]]; then
    start_rosbag_record "${bag_output}" true "${record_topics[@]}"
  fi
  # 起動順序を固定したい検証向けに、bag再生だけを実時間で遅延させる。
  if [[ "${bag_start_delay}" != "0" && "${bag_start_delay}" != "0.0" ]]; then
    sleep "${bag_start_delay}"
  fi
  "${play_cmd[@]}" &
  ROSBAG_PLAY_PID=$!
  set +e
  wait "${ROSBAG_PLAY_PID}"
  play_status=$?
  set -e

  # bag再生完了後は既定で短い猶予を置いて終了し、明示指定時だけSLAM/recordを継続する。
  if [[ "${play_status}" -eq 0 ]]; then
    if [[ "${auto_exit}" == "true" ]]; then
      echo "Rosbag playback finished. Stopping SLAM after ${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}s..." >&2
      sleep "${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}"
    else
      set +e
      wait "${SLAM_PID}"
      slam_status=$?
      set -e
      return "${slam_status}"
    fi
  fi

  return "${play_status}"
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
      --imu-type=*)
        launch_args+=("imu_type:=${1#*=}")
        ;;
      --imu-type)
        shift
        launch_args+=("imu_type:=$(require_value --imu-type "${1:-}")")
        ;;
      --imu-acceleration-unit=*)
        launch_args+=("imu_acceleration_unit:=${1#*=}")
        ;;
      --imu-acceleration-unit)
        shift
        launch_args+=("imu_acceleration_unit:=$(require_value --imu-acceleration-unit "${1:-}")")
        ;;
      --imu-acceleration-scale=*)
        launch_args+=("imu_acceleration_scale:=${1#*=}")
        ;;
      --imu-acceleration-scale)
        shift
        launch_args+=("imu_acceleration_scale:=$(require_value --imu-acceleration-scale "${1:-}")")
        ;;
      --imu-frequency=*)
        launch_args+=("imu_frequency:=${1#*=}")
        ;;
      --imu-frequency)
        shift
        launch_args+=("imu_frequency:=$(require_value --imu-frequency "${1:-}")")
        ;;
      --imu-debug)
        launch_args+=("imu_debug:=true")
        ;;
      --no-imu-debug)
        launch_args+=("imu_debug:=false")
        ;;
      --deskew-mode=*)
        launch_args+=("deskew_mode:=${1#*=}")
        ;;
      --deskew-mode)
        shift
        launch_args+=("deskew_mode:=$(require_value --deskew-mode "${1:-}")")
        ;;
      --force-zero-offset-time)
        launch_args+=("force_zero_offset_time:=true")
        ;;
      --no-force-zero-offset-time)
        launch_args+=("force_zero_offset_time:=false")
        ;;
      --wait-for-imu-initialization)
        launch_args+=("wait_for_imu_initialization:=true")
        ;;
      --no-wait-for-imu-initialization)
        launch_args+=("wait_for_imu_initialization:=false")
        ;;
      --use-imu-preintegration-initial-guess)
        launch_args+=("use_imu_preintegration_initial_guess:=true")
        ;;
      --no-use-imu-preintegration-initial-guess)
        launch_args+=("use_imu_preintegration_initial_guess:=false")
        ;;
      --use-imu-translation-initial-guess)
        launch_args+=("use_imu_translation_initial_guess:=true")
        ;;
      --no-use-imu-translation-initial-guess)
        launch_args+=("use_imu_translation_initial_guess:=false")
        ;;
      --use-imu-rotation-initial-guess)
        launch_args+=("use_imu_rotation_initial_guess:=true")
        ;;
      --no-use-imu-rotation-initial-guess)
        launch_args+=("use_imu_rotation_initial_guess:=false")
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
      --rviz-config=*)
        launch_args+=("rviz_config:=${1#*=}")
        ;;
      --rviz-config)
        shift
        launch_args+=("rviz_config:=$(require_value --rviz-config "${1:-}")")
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
      bag_output="$(default_bag_output)"
    fi
  fi
  if [[ "${record_bag}" == "true" ]]; then
    ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}" &
    # 明示指定があればそのtopicだけを記録し、未指定時は全topicを記録する。
    start_rosbag_record "${bag_output}" true "${bag_topics[@]}"
    wait
    return $?
  fi
  ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}"
}

RECORD_BAG=false
BAG_OUTPUT=""
BAG_TOPICS=()
BAG_PLAY=""
BAG_PLAY_REQUESTED=false
BAG_PLAY_RATE="1.0"
BAG_START_DELAY="0"
BAG_START_OFFSET="0"
BAG_AUTO_EXIT=true

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
    --bag-play=*)
      BAG_PLAY_REQUESTED=true
      BAG_PLAY="${1#*=}"
      ;;
    --bag-play)
      BAG_PLAY_REQUESTED=true
      if [[ $# -gt 1 && "${2:-}" != --* ]]; then
        shift
        BAG_PLAY="${1}"
      else
        BAG_PLAY=""
      fi
      ;;
    --bag-play-rate=*)
      BAG_PLAY_RATE="${1#*=}"
      ;;
    --bag-play-rate)
      shift
      BAG_PLAY_RATE="$(require_value --bag-play-rate "${1:-}")"
      ;;
    --bag-start-delay=*)
      BAG_START_DELAY="${1#*=}"
      ;;
    --bag-start-delay)
      shift
      BAG_START_DELAY="$(require_value --bag-start-delay "${1:-}")"
      ;;
    --bag-start-offset=*)
      BAG_START_OFFSET="${1#*=}"
      ;;
    --bag-start-offset)
      shift
      BAG_START_OFFSET="$(require_value --bag-start-offset "${1:-}")"
      ;;
    --auto-stop-after-bag-play=*)
      AUTO_STOP_AFTER_BAG_PLAY_SECONDS="${1#*=}"
      ;;
    --auto-stop-after-bag-play)
      shift
      AUTO_STOP_AFTER_BAG_PLAY_SECONDS="$(require_value --auto-stop-after-bag-play "${1:-}")"
      ;;
    --no-auto-exit)
      BAG_AUTO_EXIT=false
      ;;
    --bag-loop)
      echo "--bag-loop has been removed. Use --no-auto-exit if you need to keep SLAM running after one playback." >&2
      exit 2
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
  if [[ "${BAG_PLAY_REQUESTED}" == "true" ]]; then
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

if [[ "${BAG_PLAY_REQUESTED}" == "true" ]]; then
  for arg in "${FORWARD_ARGS[@]}"; do
    if [[ "${arg}" == "--no-use-sim-time" ]]; then
      echo "--bag-play always runs with use_sim_time=true; --no-use-sim-time cannot be used." >&2
      exit 2
    fi
  done
  source_sim_slam_environment
  if [[ -z "${BAG_PLAY}" ]]; then
    BAG_PLAY="$(latest_bag_for_prefix sim)"
    echo "Using latest simulation rosbag: ${BAG_PLAY}" >&2
  fi
  if [[ ! -e "${BAG_PLAY}" ]]; then
    echo "Bag path not found: ${BAG_PLAY}" >&2
    exit 1
  fi
  if [[ "${RECORD_BAG}" == "true" && -z "${BAG_OUTPUT}" ]]; then
    BAG_OUTPUT="$(default_bag_output)"
  fi
  run_bag_play_lio_sam "${BAG_PLAY}" "${BAG_PLAY_RATE}" "${BAG_START_OFFSET}" "${BAG_START_DELAY}" "${BAG_AUTO_EXIT}" "${RECORD_BAG}" "${BAG_OUTPUT}" "${BAG_TOPICS[@]}"
  exit $?
fi

if [[ "${RECORD_BAG}" == "true" ]]; then
  source_sim_slam_environment
  if [[ -z "${BAG_OUTPUT}" ]]; then
    BAG_OUTPUT="$(default_bag_output)"
  fi
  run_recorded_lio_sam "${BAG_OUTPUT}" "${BAG_TOPICS[@]}"
  exit $?
fi

# 通常時の汎用入口はbackend単体起動scriptへの互換ラッパーとして扱う。
if [[ "${#BAG_TOPICS[@]}" -gt 0 || -n "${BAG_OUTPUT}" ]]; then
  mapfile -t PASSTHROUGH_ARGS < <(build_passthrough_args)
  exec bash "${SCRIPT_DIR}/run_lio_sam.sh" "${PASSTHROUGH_ARGS[@]}"
fi

exec bash "${SCRIPT_DIR}/run_lio_sam.sh" "${FORWARD_ARGS[@]}"
