#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
SETUP_RUNTIME_SCRIPT="${WORKSPACE_ROOT}/install/setup.sh"
SETUP_SIMULATION_SCRIPT="${SIM_ROOT}/install/setup.sh"
LIDAR_PATTERN_DIR="${SIM_ROOT}/ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
THIRD_PARTY_UNDERLAY_SETUP="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
FORWARD_ARGS=()
RUN_MODE=""
PCD_MAP_PATH=""
SIM_MODE=false
DEFAULT_SIM_PARAMS_FILE="${WORKSPACE_ROOT}/ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360_sim.yaml"
ROSBAG_PID=""
ROSBAG_PLAY_PID=""
SLAM_PID=""
ROSBAG_ROOT="${WORKSPACE_ROOT}/outputs/rosbag2"
CLOUD_MAP_ROOT="${WORKSPACE_ROOT}/outputs/cloud_map"
SHUTDOWN_SIGNAL_RECEIVED=false
SHUTDOWN_SIGNAL_NAME=""
SHUTDOWN_SIGNAL_COUNT=0
CLEANUP_RUNNING=false
MAP_SAVE_IN_PROGRESS=false
MAP_SAVE_ATTEMPTED=false
MAP_SAVE_SUCCEEDED=false
MAP_SAVE_PID=""
MAP_SAVE_CANCEL_REQUESTED=false
AUTO_SAVE_PCD_MAP_ON_SHUTDOWN=true
# bag再生ではSLAM入力に必要なLiDAR/IMUと静的TFだけを流し、sim由来の動的/tf競合を避ける。
DEFAULT_BAG_PLAY_TOPICS=(
  /tf_static
  /lidar1/livox/lidar
  /lidar1/livox/imu
  /lidar2/livox/lidar
  /lidar2/livox/imu
  /lidar3/livox/lidar
  /lidar3/livox/imu
  /lidar4/livox/lidar
  /lidar4/livox/imu
)
# SLAM収録の既定topicは後段解析に必要な出力に絞り、rosbag肥大化とDDS負荷を抑える。
DEFAULT_RECORD_BAG_TOPICS=(
  /lio_sam/mapping/cloud_registered_hybrid
  /lio_sam/mapping/odometry
  /lio_sam/mapping/path
  /clock
  /tf_static
)
AUTO_STOP_AFTER_BAG_PLAY_SECONDS="${AUTO_STOP_AFTER_BAG_PLAY_SECONDS:-30}"
WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS="${WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS:-300}"
WAIT_FOR_CLOUD_QUEUE_DRAIN_POLL_SECONDS="${WAIT_FOR_CLOUD_QUEUE_DRAIN_POLL_SECONDS:-1}"
WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS="${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS:-30}"
WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS="${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS:-30}"
SAVE_PCD_MAP_TIMEOUT_SECONDS="${SAVE_PCD_MAP_TIMEOUT_SECONDS:-300}"
PROCESS_STOP_GRACE_SECONDS="15"
PROCESS_STOP_TERM_SECONDS="5"
ROSBAG_STOP_GRACE_SECONDS="60"

usage() {
  cat <<'EOF'
Usage: bash scripts/run_slam.sh {map|loc} [OPTIONS]

Modes:
  map               Build and save a PCD map.
  loc               Localize against a fixed PCD map.

Options:
  --sim              Launch Gazebo simulation and LIO-SAM together.
                     Supported only in map mode.
  --build            Run one-time environment setup before starting SLAM.
  --clean-build      Remove target workspace build artifacts, then run setup.
                     Implies --build.
  --record-bag       Record default SLAM output topics during SLAM execution.
  --bag-output PATH  Set rosbag output directory or prefix.
  --bag-topics CSV   Record only the given comma-separated topics.
                     Default: /lio_sam/mapping/cloud_registered_hybrid,/lio_sam/mapping/odometry,/lio_sam/mapping/path,/clock,/tf_static
  --bag-play [PATH]  Play a recorded rosbag and run LIO-SAM without Gazebo.
                     If PATH is omitted, the latest outputs/rosbag2/sim_* bag is used.
                     Only LiDAR/IMU topics and /tf_static are replayed.
  --bag-play-rate N  Set rosbag playback rate. Default: 1.0
  --bag-start-delay SEC
                     Delay rosbag playback start by the given seconds.
  --bag-start-offset SEC
                      Start rosbag playback after the given offset.
  --auto-stop-after-bag-play SEC
                       Seconds to keep SLAM running after cloudQueue drains. Default: 30.
  --cloud-queue-drain-timeout SEC
                        Max seconds to wait for LIO-SAM cloudQueue drain after bag playback.
                        Default: 300. Use 0 to wait without timeout.
  --no-save-map-on-shutdown
                       Do not auto-save the PCD map when mapping mode exits.
  --no-auto-exit     Keep SLAM and recording running after rosbag playback finishes.
  --rviz-config PATH Use a workspace RViz config file for LIO-SAM.
  --pcd [PATH]       Fixed PCD map path for loc mode.
                     If PATH is omitted, the latest outputs/cloud_map/*.pcd is used.
  --config PATH      Use a LIO-SAM parameter YAML. SLAM behavior/performance settings live there.
  --force-zero-offset-time
                     Force simulated Livox point offset_time to zero. Default for --sim.
  --no-force-zero-offset-time
                     Preserve simulated Livox point offset_time.
  -h, --help         Show this help.

Examples:
  bash scripts/run_slam.sh map --no-rviz
  bash scripts/run_slam.sh map --sim --lite --no-gui
  bash scripts/run_slam.sh loc --no-rviz
  bash scripts/run_slam.sh loc --pcd outputs/cloud_map/map.pcd --no-rviz
  bash scripts/run_slam.sh map --config ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml
EOF
}

reject_slam_behavior_option() {
  local option="$1"

  echo "${option} changes SLAM behavior/performance and is no longer a CLI option." >&2
  echo "Move this setting into the YAML passed with --config." >&2
  echo "Example configs:" >&2
  echo "  ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml" >&2
  echo "  ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360_sim.yaml" >&2
  exit 2
}

reject_fusion_option() {
  local option="$1"

  echo "${option} changes multi-LiDAR imageProjection wiring and is no longer a run_slam option." >&2
  echo "Move this setting into ros2_ws/src/ai_ship_robot_slam/config/multi_lidar_fusion.yaml." >&2
  exit 2
}

reject_pointcloud2_adapter_option() {
  local option="$1"

  echo "${option} has been removed because the SLAM input path is CustomMsg-only." >&2
  echo "Use multi_lidar_fusion.yaml and input_custom_topics for LiDAR input wiring." >&2
  exit 2
}

has_config_arg() {
  local index=0

  for ((index = 0; index < ${#FORWARD_ARGS[@]}; index++)); do
    case "${FORWARD_ARGS[index]}" in
      --config|--config=*)
        return 0
        ;;
    esac
  done
  return 1
}

build_requested_in_args() {
  local arg=""

  # top-levelでbuild指定を先に検出し、後続のbag解決やsource処理をbuild完了後へ送る。
  for arg in "$@"; do
    case "${arg}" in
      --build|--clean-build)
        return 0
        ;;
    esac
  done
  return 1
}

clean_build_requested_in_args() {
  local arg=""

  # clean-buildは通常buildより優先し、artifact削除を伴う再生成を一度だけ実行する。
  for arg in "$@"; do
    if [[ "${arg}" == "--clean-build" ]]; then
      return 0
    fi
  done
  return 1
}

build_runtime_workspace_if_requested() {
  local args=("$@")

  if ! build_requested_in_args "${args[@]}"; then
    return 0
  fi

  source_sim_slam_environment false

  # bag再生前の待機やoverlay読込より先にruntime buildを完了させ、未生成install参照を避ける。
  if clean_build_requested_in_args "${args[@]}"; then
    bash "${SETUP_RUNTIME_SCRIPT}" --clean-build
  else
    bash "${SETUP_RUNTIME_SCRIPT}"
  fi
}

map_saver_requested() {
  local arg=""

  for arg in "${FORWARD_ARGS[@]}"; do
    if [[ "${arg}" == "--map" ]]; then
      return 0
    fi
  done
  return 1
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
    echo "Run bash install/install_third_party.sh && bash scripts/run_slam.sh map --sim --build or --clean-build." >&2
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
      || ! source_overlay_if_current "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" \
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

  # wrapper専用引数はここで除去し、LIO-SAM launchに関係するオプションだけを残す。
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
      --build|--clean-build)
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

  # topic無指定時は既定のSLAM出力だけを記録し、不要な入力topicや大容量debug topicを避ける。
  if [[ "${#topics[@]}" -eq 0 ]]; then
    topics=("${DEFAULT_RECORD_BAG_TOPICS[@]}")
  fi
  record_cmd+=("${topics[@]}")
  echo "Rosbag output: ${output_path}" >&2
  echo "Recording rosbag topics: ${topics[*]}" >&2
  # rosbag recorderも別プロセスグループに置き、Ctrl+C時のcache flush順序をshell側で制御する。
  setsid "${record_cmd[@]}" &
  ROSBAG_PID=$!
}

default_bag_output() {
  # run_slam経由の収録は入力形態に関係なくslam prefixへ統一する。
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

latest_pcd_map() {
  local candidate_file=""
  local newest_file=""

  shopt -s nullglob
  for candidate_file in "${CLOUD_MAP_ROOT}"/*.pcd; do
    if [[ -z "${newest_file}" || "${candidate_file}" -nt "${newest_file}" ]]; then
      newest_file="${candidate_file}"
    fi
  done
  shopt -u nullglob

  # 保存済みPCDだけを候補にし、locモードで明示指定が無い場合の既定mapを決める。
  if [[ -z "${newest_file}" ]]; then
    echo "No PCD map found in ${CLOUD_MAP_ROOT}." >&2
    exit 1
  fi

  printf '%s' "${newest_file}"
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
  local root_pgid=""

  root_pgid="$(ps -o pgid= -p "${root_pid}" 2>/dev/null | tr -d '[:space:]' || true)"
  if [[ -n "${root_pgid}" && "${root_pgid}" != "$(ps -o pgid= -p "$$" 2>/dev/null | tr -d '[:space:]' || true)" ]]; then
    # 管理対象は別プロセスグループで起動するため、子孫列挙よりPGID宛てsignalを優先する。
    kill "-${signal_name}" "-${root_pgid}" 2>/dev/null || true
    return 0
  fi

  mapfile -t child_pids < <(collect_child_pids "${root_pid}")
  for child_pid in "${child_pids[@]}"; do
    kill "-${signal_name}" "${child_pid}" 2>/dev/null || true
  done
  kill "-${signal_name}" "${root_pid}" 2>/dev/null || true
}

print_pcd_map_save_start_banner() {
  local save_reason="${1:-manual}"

  echo >&2
  echo "======================================================================" >&2
  echo "PCDマップ保存を開始しました" >&2
  if [[ "${save_reason}" == "shutdown" ]]; then
    echo "SLAM停止要求を受け付けたため、終了前にPCDマップを保存しています。" >&2
  else
    echo "bag再生完了後の自動停止前にPCDマップを保存しています。" >&2
  fi
  echo "大きなマップでは数分かかる場合があります。" >&2
  echo "保存を中断して終了するには、もう一度 Ctrl+C を押してください。" >&2
  echo "======================================================================" >&2
  echo >&2
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
  local map_save_status=0

  # EXIT/TERM/INT が重なっても停止処理を一度だけ実行し、二重saveや二重killを避ける。
  if [[ "${CLEANUP_RUNNING}" == "true" ]]; then
    return 0
  fi
  CLEANUP_RUNNING=true

  # mapping終了時だけ明示saveを挟み、無効化指定時は停止だけを優先して終了経路を単純化する。
  if [[ "${RUN_MODE}" == "map" && "${AUTO_SAVE_PCD_MAP_ON_SHUTDOWN}" == "true" && -n "${SLAM_PID}" && "${MAP_SAVE_ATTEMPTED}" != "true" ]]; then
    set +e
    save_pcd_map_if_requested "shutdown"
    map_save_status=$?
    set -e
    if [[ "${map_save_status}" -ne 0 ]]; then
      echo "Proceeding to SLAM shutdown even though explicit PCD map save failed." >&2
    fi
  fi

  # bag再生終了時にSLAMも止め、逆に終了時はrecord/play側も確実に片付ける。
  stop_background_process "${ROSBAG_PID}" "rosbag recorder" "${ROSBAG_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  # bag再生プロセスはrecordとは別PIDで持ち、同時利用時も両方を確実に停止する。
  stop_background_process "${ROSBAG_PLAY_PID}" "rosbag player" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  stop_background_process "${SLAM_PID}" "SLAM" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
}

handle_shutdown_signal() {
  local signal_name="$1"
  local signal_exit_code=143

  if [[ "${signal_name}" == "INT" ]]; then
    signal_exit_code=130
  fi

  SHUTDOWN_SIGNAL_COUNT=$((SHUTDOWN_SIGNAL_COUNT + 1))
  SHUTDOWN_SIGNAL_RECEIVED=true
  SHUTDOWN_SIGNAL_NAME="${signal_name}"

  # save中の再シグナルはユーザーの中断意思として扱い、service callのプロセスグループへ伝播する。
  if [[ "${MAP_SAVE_IN_PROGRESS}" == "true" ]]; then
    MAP_SAVE_CANCEL_REQUESTED=true
    echo "${signal_name} received while PCD map save is in progress. Cancelling PCD map save..." >&2
    if [[ -n "${MAP_SAVE_PID}" ]]; then
      signal_process_tree INT "${MAP_SAVE_PID}"
    fi
    return 0
  fi

  # 初回シグナルでは、map保存を伴う停止へ入ることを明示してから cleanup へ流す。
  if [[ "${SHUTDOWN_SIGNAL_COUNT}" -eq 1 ]]; then
    if [[ "${RUN_MODE}" == "map" && "${AUTO_SAVE_PCD_MAP_ON_SHUTDOWN}" == "true" ]]; then
      echo "${signal_name} received. Mapping shutdown will save the PCD map before stopping SLAM." >&2
      echo "Press Ctrl+C again during the save to cancel the PCD map save." >&2
    elif [[ "${RUN_MODE}" == "map" ]]; then
      echo "${signal_name} received. Stopping mapping without automatic PCD map save." >&2
    else
      echo "${signal_name} received. Stopping SLAM and related background processes." >&2
    fi
    exit "${signal_exit_code}"
  fi

  # cleanup待ち中の再シグナルでは現在の段階だけを伝え、無駄な再押下を抑止する。
  if [[ "${CLEANUP_RUNNING}" == "true" ]]; then
    echo "${signal_name} received again while shutdown is already in progress. Waiting for current stop sequence to finish." >&2
  else
    echo "${signal_name} received again. Shutdown has already been requested; waiting for cleanup to start." >&2
  fi
  return 0
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

wait_for_lio_sam_startup() {
  local started_seconds=${SECONDS}
  local elapsed_seconds=0
  local last_log_seconds=0
  local node_list=""
  local required_node=""
  local missing_nodes=()
  local required_nodes=(
    "/slam_reference_lidar_static_tf_node"
    "/lio_sam_imuPreintegration"
    "/lio_sam_imageProjection"
    "/lio_sam_featureExtraction"
    "/lio_sam_mapOptimization"
  )

  # localizationでは固定PCD読み込み完了後のlocalizerも待ち、bag先頭のscan取りこぼしを避ける。
  if [[ "${RUN_MODE}" == "loc" ]]; then
    required_nodes+=("/pcd_localization_node")
  fi

  echo "Waiting for LIO-SAM startup before rosbag playback..." >&2
  while true; do
    if [[ -n "${SLAM_PID}" ]] && ! kill -0 "${SLAM_PID}" 2>/dev/null; then
      echo "LIO-SAM exited before startup completed." >&2
      return 1
    fi

    # ROS graphに必須ノードが全て見えてからbagを流し、起動直後のIMU/topic取りこぼしを避ける。
    node_list="$(timeout 5s ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)"
    missing_nodes=()
    for required_node in "${required_nodes[@]}"; do
      if ! grep -Fxq "${required_node}" <<< "${node_list}"; then
        missing_nodes+=("${required_node}")
      fi
    done

    if [[ "${#missing_nodes[@]}" -eq 0 ]]; then
      echo "LIO-SAM startup completed." >&2
      return 0
    fi

    elapsed_seconds=$((SECONDS - started_seconds))
    if [[ "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" != "0" &&
          "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" != "0.0" &&
          "${elapsed_seconds}" -ge "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" ]]; then
      echo "Timed out waiting for LIO-SAM startup after ${elapsed_seconds}s." >&2
      echo "Missing ROS nodes: ${missing_nodes[*]}" >&2
      echo "Current ROS nodes:" >&2
      echo "${node_list}" >&2
      return 1
    fi

    if (( SECONDS - last_log_seconds >= 5 )); then
      echo "Still waiting for LIO-SAM nodes: ${missing_nodes[*]}" >&2
      last_log_seconds=${SECONDS}
    fi
    sleep 0.2
  done
}

is_bag_play_sensor_topic() {
  local topic="$1"

  [[ "${topic}" == *"/livox/lidar" || "${topic}" == *"/livox/imu" ]]
}

topic_has_subscriber() {
  local topic="$1"
  local topic_info=""

  # ROS graph上の購読数だけを見て、bag再生前に入力待機状態へ入ったことを確認する。
  topic_info="$(timeout 5s ros2 topic info "${topic}" --no-daemon --spin-time 2 2>/dev/null || true)"
  [[ "${topic_info}" =~ Subscription[[:space:]]count:[[:space:]]([1-9][0-9]*) ]]
}

wait_for_bag_play_sensor_subscriber() {
  local started_seconds=${SECONDS}
  local elapsed_seconds=0
  local last_log_seconds=0
  local topic=""
  local sensor_topics=()

  # /tf_staticや後段のfiltered topicは除外し、bagに含まれるraw LiDAR/IMU入力だけを待機対象にする。
  for topic in "$@"; do
    if is_bag_play_sensor_topic "${topic}"; then
      sensor_topics+=("${topic}")
    fi
  done

  if [[ "${#sensor_topics[@]}" -eq 0 ]]; then
    echo "No LiDAR/IMU rosbag playback topics configured; skipping subscriber wait." >&2
    return 0
  fi

  echo "Waiting for a LiDAR/IMU subscriber before rosbag playback..." >&2
  while true; do
    if [[ -n "${SLAM_PID}" ]] && ! kill -0 "${SLAM_PID}" 2>/dev/null; then
      echo "LIO-SAM exited before LiDAR/IMU subscriber became ready." >&2
      return 1
    fi

    # どれか1つの入力topicにsubscriberが見えれば、SLAM側が受信待機に入ったとみなす。
    for topic in "${sensor_topics[@]}"; do
      if topic_has_subscriber "${topic}"; then
        echo "LiDAR/IMU subscriber is ready on ${topic}." >&2
        return 0
      fi
    done

    elapsed_seconds=$((SECONDS - started_seconds))
    if [[ "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" != "0" &&
          "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" != "0.0" &&
          "${elapsed_seconds}" -ge "${WAIT_FOR_SLAM_STARTUP_TIMEOUT_SECONDS}" ]]; then
      echo "Timed out waiting for a LiDAR/IMU subscriber after ${elapsed_seconds}s." >&2
      echo "Checked topics: ${sensor_topics[*]}" >&2
      return 1
    fi

    if (( SECONDS - last_log_seconds >= 5 )); then
      echo "Still waiting for a LiDAR/IMU subscriber on: ${sensor_topics[*]}" >&2
      last_log_seconds=${SECONDS}
    fi
    sleep 0.2
  done
}

wait_for_lio_sam_cloud_queue_empty() {
  local service_name="/lio_sam/deskew/is_cloud_queue_empty"
  local service_type="lio_sam/srv/SaveMap"
  local service_request="{resolution: 0.0, destination: ''}"
  local call_output=""
  local call_status=0
  local started_seconds=${SECONDS}
  local last_log_seconds=0
  local elapsed_seconds=0

  echo "Rosbag playback finished. Waiting for LIO-SAM imageProjection cloudQueue to drain..." >&2
  while true; do
    if [[ -n "${SLAM_PID}" ]] && ! kill -0 "${SLAM_PID}" 2>/dev/null; then
      echo "LIO-SAM exited before cloudQueue drain completed." >&2
      return 1
    fi

    set +e
    call_output="$(timeout 5s ros2 service call "${service_name}" "${service_type}" "${service_request}" 2>&1)"
    call_status=$?
    set -e

    if [[ "${call_status}" -eq 0 ]]; then
      if [[ "${call_output}" == *"success=True"* || "${call_output}" == *"success: true"* || "${call_output}" == *"success: True"* ]]; then
        echo "LIO-SAM imageProjection cloudQueue drained." >&2
        return 0
      fi
      if (( SECONDS - last_log_seconds >= 10 )); then
        echo "Still waiting for LIO-SAM cloudQueue drain: ${call_output//$'\n'/ }" >&2
        last_log_seconds=${SECONDS}
      fi
    else
      if (( SECONDS - last_log_seconds >= 10 )); then
        echo "Waiting for cloudQueue status service ${service_name}: ${call_output//$'\n'/ }" >&2
        last_log_seconds=${SECONDS}
      fi
    fi

    elapsed_seconds=$((SECONDS - started_seconds))
    if [[ "${WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS}" != "0" &&
          "${WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS}" != "0.0" &&
          "${elapsed_seconds}" -ge "${WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS}" ]]; then
      echo "Timed out waiting for LIO-SAM cloudQueue drain after ${elapsed_seconds}s." >&2
      return 1
    fi

    sleep "${WAIT_FOR_CLOUD_QUEUE_DRAIN_POLL_SECONDS}"
  done
}

wait_for_save_pcd_map_service() {
  local service_name="/save_pcd_map"
  local started_seconds=${SECONDS}
  local elapsed_seconds=0
  local last_log_seconds=0
  local service_list=""

  echo "Waiting for PCD map saver service ${service_name}..." >&2
  while true; do
    if [[ -n "${SLAM_PID}" ]] && ! kill -0 "${SLAM_PID}" 2>/dev/null; then
      echo "LIO-SAM exited before PCD map saver service became available." >&2
      return 1
    fi

    # ROS graph上でmap saver serviceの起動を確認してから保存要求を出す。
    service_list="$(timeout 5s ros2 service list --no-daemon 2>/dev/null || true)"
    if grep -Fxq "${service_name}" <<< "${service_list}"; then
      return 0
    fi

    elapsed_seconds=$((SECONDS - started_seconds))
    if [[ "${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS}" != "0" &&
          "${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS}" != "0.0" &&
          "${elapsed_seconds}" -ge "${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS}" ]]; then
      echo "Timed out waiting for PCD map saver service after ${elapsed_seconds}s." >&2
      return 1
    fi

    if (( SECONDS - last_log_seconds >= 5 )); then
      echo "Still waiting for PCD map saver service ${service_name}." >&2
      last_log_seconds=${SECONDS}
    fi
    sleep 0.5
  done
}

save_pcd_map_if_requested() {
  local save_reason="${1:-manual}"
  local service_name="/save_pcd_map"
  local service_type="std_srvs/srv/Trigger"
  local service_request="{}"
  local call_output=""
  local call_status=0
  local call_cmd=(ros2 service call "${service_name}" "${service_type}" "${service_request}")
  local output_file=""
  local saved_map_path=""

  if [[ "${RUN_MODE}" != "map" ]]; then
    return 0
  fi
  if [[ "${MAP_SAVE_ATTEMPTED}" == "true" ]]; then
    if [[ "${MAP_SAVE_SUCCEEDED}" == "true" ]]; then
      echo "PCD map save was already completed earlier; skipping duplicate save request." >&2
      return 0
    fi
    echo "PCD map save was already attempted earlier; skipping duplicate save request." >&2
    return 1
  fi

  # save service待機以降は明示的に save試行中として扱い、重複要求と再シグナル中断を防ぐ。
  MAP_SAVE_ATTEMPTED=true
  if ! wait_for_save_pcd_map_service; then
    return 1
  fi

  print_pcd_map_save_start_banner "${save_reason}"

  MAP_SAVE_IN_PROGRESS=true
  MAP_SAVE_CANCEL_REQUESTED=false
  output_file="$(mktemp -t ai_ship_robot_pcd_map_save.XXXXXX.log)"
  set +e
  if [[ "${SAVE_PCD_MAP_TIMEOUT_SECONDS}" == "0" || "${SAVE_PCD_MAP_TIMEOUT_SECONDS}" == "0.0" ]]; then
    setsid "${call_cmd[@]}" >"${output_file}" 2>&1 &
    MAP_SAVE_PID=$!
  else
    setsid timeout --foreground "${SAVE_PCD_MAP_TIMEOUT_SECONDS}s" "${call_cmd[@]}" >"${output_file}" 2>&1 &
    MAP_SAVE_PID=$!
  fi
  wait "${MAP_SAVE_PID}"
  call_status=$?
  set -e
  call_output="$(<"${output_file}")"
  rm -f "${output_file}"
  MAP_SAVE_PID=""
  MAP_SAVE_IN_PROGRESS=false

  if [[ "${MAP_SAVE_CANCEL_REQUESTED}" == "true" || "${call_status}" -eq 130 || "${call_status}" -eq 124 ]]; then
    echo "----------------------------------------------------------------------" >&2
    echo "PCD map save cancelled by user." >&2
    echo "----------------------------------------------------------------------" >&2
    return 1
  fi

  # service callはsuccess=falseでも終了コード0になり得るため、応答本文も確認する。
  if [[ "${call_status}" -eq 0 &&
        ( "${call_output}" == *"success=True"* ||
         "${call_output}" == *"success: true"* ||
          "${call_output}" == *"success: True"* ) ]]; then
    MAP_SAVE_SUCCEEDED=true
    # service応答には保存先とframeが入るため、完了ログではPCDパスだけを目立つ形で再掲する。
    if [[ "${call_output}" =~ ([^[:space:]\"\']+\.pcd) ]]; then
      saved_map_path="${BASH_REMATCH[1]}"
    fi
    echo "----------------------------------------------------------------------" >&2
    echo "PCDマップ保存が完了しました。" >&2
    if [[ -n "${saved_map_path}" ]]; then
      echo "保存先PCDファイル: ${saved_map_path}" >&2
    fi
    echo "PCD map save service response: ${call_output//$'\n'/ }" >&2
    echo "----------------------------------------------------------------------" >&2
    echo "Continuing with the remaining shutdown sequence." >&2
    return 0
  fi

  echo "----------------------------------------------------------------------" >&2
  echo "PCD map save failed: ${call_output//$'\n'/ }" >&2
  echo "----------------------------------------------------------------------" >&2
  echo "Continuing with SLAM shutdown. The shutdown fallback saver may still attempt a final save." >&2
  return 1
}

run_managed_slam_launch() {
  local slam_args=("$@")
  local slam_status=0

  # 通常起動もbackground管理へ寄せ、終了signal時に save service を呼ぶ余地を script 側へ残す。
  run_slam_launch "${slam_args[@]}" &
  SLAM_PID=$!
  ensure_process_started "${SLAM_PID}" "SLAM"

  set +e
  wait "${SLAM_PID}"
  slam_status=$?
  set -e
  return "${slam_status}"
}

run_managed_sim_launch() {
  local launch_args=("$@")
  local sim_status=0

  # sim launch も同じ supervision 配下に置き、map保存と launch tree 停止を同じ cleanup で扱う。
  setsid ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}" &
  SLAM_PID=$!
  ensure_process_started "${SLAM_PID}" "simulation SLAM"

  set +e
  wait "${SLAM_PID}"
  sim_status=$?
  set -e
  return "${sim_status}"
}

run_slam_launch() {
  local build_workspace=false
  local clean_build_workspace=false
  local launch_file="lio_sam.launch.py"
  local launch_args=()

  while [[ $# -gt 0 ]]; do
    # multi-LiDAR入力配線はmulti_lidar_fusion.yamlへ集約し、CLIからの上書きを禁止する。
    if [[ "$1" =~ ^--lidar([0-9]+)-points=(.+)$ ]]; then
      reject_fusion_option "--lidar${BASH_REMATCH[1]}-points"
    fi
    if [[ "$1" =~ ^--lidar([0-9]+)-points$ ]]; then
      reject_fusion_option "--lidar${BASH_REMATCH[1]}-points"
    fi

    case "$1" in
      --build)
        build_workspace=true
        ;;
      --clean-build)
        clean_build_workspace=true
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
        reject_fusion_option "--fusion-config"
        ;;
      --fusion-config)
        reject_fusion_option "--fusion-config"
        ;;
      --input-points=*)
        reject_fusion_option "--input-points"
        ;;
      --input-points)
        reject_fusion_option "--input-points"
        ;;
      --points=*)
        reject_fusion_option "--points"
        ;;
      --points)
        reject_fusion_option "--points"
        ;;
      --reference-points=*)
        reject_fusion_option "--reference-points"
        ;;
      --reference-points)
        reject_fusion_option "--reference-points"
        ;;
      --reference-lidar-frame=*)
        reject_fusion_option "--reference-lidar-frame"
        ;;
      --reference-lidar-frame)
        reject_fusion_option "--reference-lidar-frame"
        ;;
      --fused-points=*)
        reject_fusion_option "--fused-points"
        ;;
      --fused-points)
        reject_fusion_option "--fused-points"
        ;;
      --imu=*)
        launch_args+=("imu_topic:=${1#*=}")
        ;;
      --imu)
        shift
        launch_args+=("imu_topic:=$(require_value --imu "${1:-}")")
        ;;
      --imu-type=*)
        reject_slam_behavior_option "--imu-type"
        ;;
      --imu-type)
        reject_slam_behavior_option "--imu-type"
        ;;
      --imu-acceleration-unit=*)
        reject_slam_behavior_option "--imu-acceleration-unit"
        ;;
      --imu-acceleration-unit)
        reject_slam_behavior_option "--imu-acceleration-unit"
        ;;
      --imu-acceleration-scale=*)
        reject_slam_behavior_option "--imu-acceleration-scale"
        ;;
      --imu-acceleration-scale)
        reject_slam_behavior_option "--imu-acceleration-scale"
        ;;
      --imu-frequency=*)
        reject_slam_behavior_option "--imu-frequency"
        ;;
      --imu-frequency)
        reject_slam_behavior_option "--imu-frequency"
        ;;
      --imu-debug)
        reject_slam_behavior_option "--imu-debug"
        ;;
      --no-imu-debug)
        reject_slam_behavior_option "--no-imu-debug"
        ;;
      --deskew-mode=*)
        reject_slam_behavior_option "--deskew-mode"
        ;;
      --deskew-mode)
        reject_slam_behavior_option "--deskew-mode"
        ;;
      --wait-for-imu-initialization)
        reject_slam_behavior_option "--wait-for-imu-initialization"
        ;;
      --no-wait-for-imu-initialization)
        reject_slam_behavior_option "--no-wait-for-imu-initialization"
        ;;
      --use-imu-preintegration-initial-guess)
        reject_slam_behavior_option "--use-imu-preintegration-initial-guess"
        ;;
      --no-use-imu-preintegration-initial-guess)
        reject_slam_behavior_option "--no-use-imu-preintegration-initial-guess"
        ;;
      --use-imu-translation-initial-guess)
        reject_slam_behavior_option "--use-imu-translation-initial-guess"
        ;;
      --no-use-imu-translation-initial-guess)
        reject_slam_behavior_option "--no-use-imu-translation-initial-guess"
        ;;
      --use-imu-rotation-initial-guess)
        reject_slam_behavior_option "--use-imu-rotation-initial-guess"
        ;;
      --no-use-imu-rotation-initial-guess)
        reject_slam_behavior_option "--no-use-imu-rotation-initial-guess"
        ;;
      --lio-points=*)
        reject_fusion_option "--lio-points"
        ;;
      --lio-points)
        reject_fusion_option "--lio-points"
        ;;
      --lio-custom=*)
        reject_fusion_option "--lio-custom"
        ;;
      --lio-custom)
        reject_fusion_option "--lio-custom"
        ;;
      --adapter)
        reject_pointcloud2_adapter_option "--adapter"
        ;;
      --no-adapter)
        reject_pointcloud2_adapter_option "--no-adapter"
        ;;
      --fusion)
        reject_fusion_option "--fusion"
        ;;
      --no-fusion)
        reject_fusion_option "--no-fusion"
        ;;
      --map-to-odom-z|--map-to-odom-z=*)
        echo "--map-to-odom-z has been removed. Align world/odom later with an external static TF." >&2
        exit 2
        ;;
      --fusion-timestamp-scale=*)
        reject_slam_behavior_option "--fusion-timestamp-scale"
        ;;
      --fusion-timestamp-scale)
        reject_slam_behavior_option "--fusion-timestamp-scale"
        ;;
      --use-sim-time)
        launch_args+=("use_sim_time:=true")
        ;;
      --no-use-sim-time)
        launch_args+=("use_sim_time:=false")
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
      --lio-sam-package=*)
        launch_args+=("lio_sam_package:=${1#*=}")
        ;;
      --lio-sam-package)
        shift
        launch_args+=("lio_sam_package:=$(require_value --lio-sam-package "${1:-}")")
        ;;
      --lio-sam|--lio_sam|--liosam)
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

  source_sim_slam_environment false

  if [[ "${build_workspace}" == "true" ]]; then
    # 明示build時だけworkspace setupを実行し、clean指定時は同じ導線で再生成まで行う。
    if [[ "${clean_build_workspace}" == "true" ]]; then
      bash "${SETUP_RUNTIME_SCRIPT}" --clean-build
    else
      bash "${SETUP_RUNTIME_SCRIPT}"
    fi
  fi

  if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "Missing ros2_ws/install/setup.bash. Run bash install/setup.sh first." >&2
    exit 1
  fi

  source_sim_slam_environment
  export AI_SHIP_ROBOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}"

  # 実行モードごとに起動launchを固定し、map作成と固定PCD localizationの責務を分ける。
  case "${RUN_MODE}" in
    map)
      launch_file="lio_sam.launch.py"
      launch_args+=("use_map_saver:=true")
      ;;
    loc)
      launch_file="lio_sam_localization.launch.py"
      launch_args+=("pcd_map_path:=${PCD_MAP_PATH}")
      ;;
  esac

  exec setsid ros2 launch ai_ship_robot_slam "${launch_file}" "${launch_args[@]}"
}

run_recorded_lio_sam() {
  local bag_output="$1"
  local slam_args=()

  # 単体SLAM収録では既定topicを中央のrecord処理に委ね、CLI指定時だけ上書きする。
  start_rosbag_record "${bag_output}" false
  mapfile -t slam_args < <(build_passthrough_args)
  run_managed_slam_launch "${slam_args[@]}"
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
  local play_topics=("${DEFAULT_BAG_PLAY_TOPICS[@]}")
  local play_cmd=(
    ros2 bag play "${bag_path}" --clock --rate "${bag_rate}" --start-offset "${bag_offset}"
    --topics "${play_topics[@]}"
  )
  local slam_args=()
  local map_save_status=0

  mapfile -t slam_args < <(build_passthrough_args)
  if ! has_config_arg; then
    slam_args+=(--config "${DEFAULT_SIM_PARAMS_FILE}")
  fi
  run_slam_launch \
    --use-sim-time \
    --imu /lidar1/livox/imu \
    "${slam_args[@]}" &
  SLAM_PID=$!
  ensure_process_started "${SLAM_PID}" "LIO-SAM"
  wait_for_lio_sam_startup
  wait_for_bag_play_sensor_subscriber "${play_topics[@]}"
  # bag再生前にrecordを起動し、再生開始直後のtopicも取りこぼしにくくする。
  if [[ "${record_bag}" == "true" ]]; then
    start_rosbag_record "${bag_output}" true "${record_topics[@]}"
  fi
  # 起動順序を固定したい検証向けに、bag再生だけを実時間で遅延させる。
  if [[ "${bag_start_delay}" != "0" && "${bag_start_delay}" != "0.0" ]]; then
    sleep "${bag_start_delay}"
  fi
  echo "Replaying rosbag topics: ${play_topics[*]}" >&2
  # bag playerをSLAM launchと分離し、停止時に保存serviceが先に完了できる状態を保つ。
  setsid "${play_cmd[@]}" &
  ROSBAG_PLAY_PID=$!
  set +e
  wait "${ROSBAG_PLAY_PID}"
  play_status=$?
  set -e

  # bag再生完了後は既定で短い猶予を置いて終了し、明示指定時だけSLAM/recordを継続する。
  if [[ "${play_status}" -eq 0 ]]; then
    if [[ "${auto_exit}" == "true" ]]; then
      if ! wait_for_lio_sam_cloud_queue_empty; then
        echo "Proceeding to auto-stop after cloudQueue drain wait failed." >&2
      fi
      if [[ "${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}" != "0" && "${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}" != "0.0" ]]; then
        echo "Waiting ${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}s post-drain grace before SLAM stop..." >&2
        sleep "${AUTO_STOP_AFTER_BAG_PLAY_SECONDS}"
      fi
      # bag再生の auto-stop でも通常終了と同じ flag を見て、保存抑止時の挙動差をなくす。
      if [[ "${AUTO_SAVE_PCD_MAP_ON_SHUTDOWN}" == "true" ]]; then
        set +e
        save_pcd_map_if_requested
        map_save_status=$?
        set -e
      fi
    else
      set +e
      wait "${SLAM_PID}"
      slam_status=$?
      set -e
      return "${slam_status}"
    fi
  fi

  if [[ "${play_status}" -eq 0 && "${map_save_status}" -ne 0 ]]; then
    return "${map_save_status}"
  fi
  return "${play_status}"
}

run_sim_lio_sam() {
  local build_workspace=false
  local clean_build_workspace=false
  local lite_mode=false
  local robot_name_set=false
  local launch_args=()
  local record_bag=false
  local bag_output=""
  local bag_topics=()

  while [[ $# -gt 0 ]]; do
    # simulationでもmulti-LiDAR入力配線はmulti_lidar_fusion.yamlからのみ読む。
    if [[ "$1" =~ ^--lidar([0-9]+)-points=(.+)$ ]]; then
      reject_fusion_option "--lidar${BASH_REMATCH[1]}-points"
    fi
    if [[ "$1" =~ ^--lidar([0-9]+)-points$ ]]; then
      reject_fusion_option "--lidar${BASH_REMATCH[1]}-points"
    fi

    case "$1" in
      --build)
        build_workspace=true
        ;;
      --clean-build)
        clean_build_workspace=true
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
        reject_fusion_option "--fusion-config"
        ;;
      --fusion-config)
        reject_fusion_option "--fusion-config"
        ;;
      --input-points=*)
        reject_fusion_option "--input-points"
        ;;
      --input-points)
        reject_fusion_option "--input-points"
        ;;
      --points=*)
        reject_fusion_option "--points"
        ;;
      --points)
        reject_fusion_option "--points"
        ;;
      --reference-points=*)
        reject_fusion_option "--reference-points"
        ;;
      --reference-points)
        reject_fusion_option "--reference-points"
        ;;
      --reference-lidar-frame=*)
        reject_fusion_option "--reference-lidar-frame"
        ;;
      --reference-lidar-frame)
        reject_fusion_option "--reference-lidar-frame"
        ;;
      --fused-points=*)
        reject_fusion_option "--fused-points"
        ;;
      --fused-points)
        reject_fusion_option "--fused-points"
        ;;
      --imu=*)
        launch_args+=("imu_topic:=${1#*=}")
        ;;
      --imu)
        shift
        launch_args+=("imu_topic:=$(require_value --imu "${1:-}")")
        ;;
      --imu-type=*)
        reject_slam_behavior_option "--imu-type"
        ;;
      --imu-type)
        reject_slam_behavior_option "--imu-type"
        ;;
      --imu-acceleration-unit=*)
        reject_slam_behavior_option "--imu-acceleration-unit"
        ;;
      --imu-acceleration-unit)
        reject_slam_behavior_option "--imu-acceleration-unit"
        ;;
      --imu-acceleration-scale=*)
        reject_slam_behavior_option "--imu-acceleration-scale"
        ;;
      --imu-acceleration-scale)
        reject_slam_behavior_option "--imu-acceleration-scale"
        ;;
      --imu-frequency=*)
        reject_slam_behavior_option "--imu-frequency"
        ;;
      --imu-frequency)
        reject_slam_behavior_option "--imu-frequency"
        ;;
      --imu-debug)
        reject_slam_behavior_option "--imu-debug"
        ;;
      --no-imu-debug)
        reject_slam_behavior_option "--no-imu-debug"
        ;;
      --deskew-mode=*)
        reject_slam_behavior_option "--deskew-mode"
        ;;
      --deskew-mode)
        reject_slam_behavior_option "--deskew-mode"
        ;;
      --force-zero-offset-time)
        launch_args+=("force_zero_offset_time:=true")
        ;;
      --no-force-zero-offset-time)
        launch_args+=("force_zero_offset_time:=false")
        ;;
      --wait-for-imu-initialization)
        reject_slam_behavior_option "--wait-for-imu-initialization"
        ;;
      --no-wait-for-imu-initialization)
        reject_slam_behavior_option "--no-wait-for-imu-initialization"
        ;;
      --use-imu-preintegration-initial-guess)
        reject_slam_behavior_option "--use-imu-preintegration-initial-guess"
        ;;
      --no-use-imu-preintegration-initial-guess)
        reject_slam_behavior_option "--no-use-imu-preintegration-initial-guess"
        ;;
      --use-imu-translation-initial-guess)
        reject_slam_behavior_option "--use-imu-translation-initial-guess"
        ;;
      --no-use-imu-translation-initial-guess)
        reject_slam_behavior_option "--no-use-imu-translation-initial-guess"
        ;;
      --use-imu-rotation-initial-guess)
        reject_slam_behavior_option "--use-imu-rotation-initial-guess"
        ;;
      --no-use-imu-rotation-initial-guess)
        reject_slam_behavior_option "--no-use-imu-rotation-initial-guess"
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
        reject_fusion_option "--lio-points"
        ;;
      --lio-points)
        reject_fusion_option "--lio-points"
        ;;
      --lio-custom=*)
        reject_fusion_option "--lio-custom"
        ;;
      --lio-custom)
        reject_fusion_option "--lio-custom"
        ;;
      --adapter)
        reject_pointcloud2_adapter_option "--adapter"
        ;;
      --no-adapter)
        reject_pointcloud2_adapter_option "--no-adapter"
        ;;
      --fusion)
        reject_fusion_option "--fusion"
        ;;
      --no-fusion)
        reject_fusion_option "--no-fusion"
        ;;
      --fusion-timestamp-scale=*)
        reject_slam_behavior_option "--fusion-timestamp-scale"
        ;;
      --fusion-timestamp-scale)
        reject_slam_behavior_option "--fusion-timestamp-scale"
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

  # lite指定時はGazebo GUIを止め、LiDAR処理もsimulation launch側の軽量設定へ切り替える。
  launch_args+=("lite:=${lite_mode}")
  if [[ "${lite_mode}" == "true" ]]; then
    launch_args+=("gui:=false")
  fi

  # Gazebo entity名の衝突を避けるため、未指定時だけ一意な名前を使う。
  if [[ "${robot_name_set}" == "false" ]]; then
    launch_args+=("robot_name:=ai_ship_robot_lio_sam_$$")
  fi

  # simulation経由のmap作成でも、通常起動と同じPCD map saverを有効化する。
  launch_args+=("use_map_saver:=true")

  source_sim_slam_environment false

  if [[ "${build_workspace}" == "true" ]]; then
    # --simのbuildはruntimeとsimulationを順に更新し、clean指定時だけ各workspaceを再生成する。
    if [[ "${clean_build_workspace}" == "true" ]]; then
      bash "${SETUP_RUNTIME_SCRIPT}" --clean-build
      bash "${SETUP_SIMULATION_SCRIPT}" --clean-build
    else
      bash "${SETUP_RUNTIME_SCRIPT}"
      bash "${SETUP_SIMULATION_SCRIPT}"
    fi
  fi

  if [[ ! -f "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "Missing ros2_ws/install/setup.bash. Run bash install/setup.sh first." >&2
    exit 1
  fi

  if [[ ! -f "${SIM_ROOT}/ros2_ws/install/setup.bash" ]]; then
    echo "Missing sim/ros2_ws/install/setup.bash. Run bash sim/install/setup.sh first." >&2
    exit 1
  fi

  source_sim_slam_environment
  export AI_SHIP_ROBOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}"
  if [[ "${record_bag}" == "true" ]]; then
    if [[ -z "${bag_output}" ]]; then
      bag_output="$(default_bag_output)"
    fi
  fi
  if [[ "${record_bag}" == "true" ]]; then
    setsid ros2 launch ai_ship_robot_gazebo sim_lio_sam.launch.py "${launch_args[@]}" &
    SLAM_PID=$!
    ensure_process_started "${SLAM_PID}" "simulation SLAM"
    # 明示指定があればそのtopicだけを記録し、未指定時は既定topicを記録する。
    start_rosbag_record "${bag_output}" true "${bag_topics[@]}"
    wait
    return $?
  fi
  run_managed_sim_launch "${launch_args[@]}"
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
PCD_MAP_REQUESTED=false

trap cleanup_background_processes EXIT
trap 'handle_shutdown_signal INT' INT
trap 'handle_shutdown_signal TERM' TERM

# 先頭引数を実行モードとして固定し、以降のoption解析からmode名を分離する。
if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

case "$1" in
  -h|--help)
    usage
    exit 0
    ;;
  map|loc)
    RUN_MODE="$1"
    shift
    ;;
  *)
    echo "First argument must be 'map' or 'loc': $1" >&2
    usage >&2
    exit 2
    ;;
esac

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
    --pcd=*)
      PCD_MAP_REQUESTED=true
      PCD_MAP_PATH="${1#*=}"
      ;;
    --pcd)
      PCD_MAP_REQUESTED=true
      if [[ $# -gt 1 && "${2:-}" != --* ]]; then
        shift
        PCD_MAP_PATH="${1}"
      else
        PCD_MAP_PATH=""
      fi
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
    --cloud-queue-drain-timeout=*)
      WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS="${1#*=}"
      ;;
    --cloud-queue-drain-timeout)
      shift
      WAIT_FOR_CLOUD_QUEUE_DRAIN_TIMEOUT_SECONDS="$(require_value --cloud-queue-drain-timeout "${1:-}")"
      ;;
    --no-save-map-on-shutdown)
      AUTO_SAVE_PCD_MAP_ON_SHUTDOWN=false
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

# localizationは固定PCD mapを使うが、未指定時は最新の保存済みPCDを既定値にする。
if [[ "${RUN_MODE}" == "loc" && -z "${PCD_MAP_PATH}" ]]; then
  PCD_MAP_PATH="$(latest_pcd_map)"
  echo "Using latest PCD map: ${PCD_MAP_PATH}" >&2
fi
if [[ "${RUN_MODE}" == "map" && "${PCD_MAP_REQUESTED}" == "true" ]]; then
  echo "map mode does not use --pcd." >&2
  exit 2
fi
if [[ "${RUN_MODE}" == "loc" && "${SIM_MODE}" == "true" ]]; then
  echo "loc mode does not support --sim." >&2
  exit 2
fi

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

# 非sim経路ではbuild指定を最優先で処理し、後続のsourceやbag判定を新しいinstall成果物前提へ揃える。
build_runtime_workspace_if_requested "${FORWARD_ARGS[@]}"

if [[ "${BAG_PLAY_REQUESTED}" == "true" ]]; then
  for arg in "${FORWARD_ARGS[@]}"; do
    if [[ "${arg}" == "--no-use-sim-time" ]]; then
      echo "--bag-play always runs with use_sim_time=true; --no-use-sim-time cannot be used." >&2
      exit 2
    fi
  done
  source_sim_slam_environment
  export AI_SHIP_ROBOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}"
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
  export AI_SHIP_ROBOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}"
  if [[ -z "${BAG_OUTPUT}" ]]; then
    BAG_OUTPUT="$(default_bag_output)"
  fi
  run_recorded_lio_sam "${BAG_OUTPUT}" "${BAG_TOPICS[@]}"
  exit $?
fi

# 通常時はこのscript内のLIO-SAM単体起動処理を直接呼び出す。
if [[ "${#BAG_TOPICS[@]}" -gt 0 || -n "${BAG_OUTPUT}" ]]; then
  mapfile -t PASSTHROUGH_ARGS < <(build_passthrough_args)
  run_managed_slam_launch "${PASSTHROUGH_ARGS[@]}"
  exit $?
fi

mapfile -t PASSTHROUGH_ARGS < <(build_passthrough_args)
run_managed_slam_launch "${PASSTHROUGH_ARGS[@]}"
