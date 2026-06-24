#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
SETUP_SIMULATION_SCRIPT="${SIM_ROOT}/install/setup.sh"
LIDAR_PATTERN_DIR="${SIM_ROOT}/ros2_ws/src/ai_ship_robot_description/urdf/lidar/patterns"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
THIRD_PARTY_UNDERLAY_SETUP="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
ROSBAG_PID=""
SIMULATION_PID=""
DRIVE_PID=""
POINTCLOUD_BRIDGE_PID=""
ROSBAG_ROOT="${WORKSPACE_ROOT}/outputs/rosbag2"
AUTO_STOP_AFTER_DRIVE_SECONDS="5"
SIM_READY_TIMEOUT_SECONDS="300"
PROCESS_STOP_GRACE_SECONDS="15"
PROCESS_STOP_TERM_SECONDS="5"
ROSBAG_STOP_GRACE_SECONDS="60"
TEMP_WORLD_FILE=""
GAZEBO_MODEL_DIR="${SIM_ROOT}/ros2_ws/install/ai_ship_robot_description/share/gazebo_models"

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
Usage: bash sim/scripts/run_simulation.sh [OPTIONS]

Options:
  --build             Run one-time environment setup before launching simulation.
  --clean-build       Remove target workspace build artifacts, then run setup.
                      Implies --build.
  --lite              Disable Gazebo Classic GUI and default LiDAR rays to quarter resolution.
  --gui               Enable Gazebo Classic GUI.
  --no-gui            Disable Gazebo Classic GUI.
  --rviz              Enable RViz2.
  --no-rviz           Disable RViz2.
  --odom-tf           Publish Gazebo odom -> base_footprint TF. Default: enabled.
  --no-odom-tf        Do not publish Gazebo odom -> base_footprint TF.
  -4, --quarter-resolution
                      Use quarter LiDAR sample density.
  -2, --half-resolution
                      Use half LiDAR sample density.
  -1, --full-resolution
                      Use full LiDAR sample counts.
  --world PATH        Use a custom Gazebo Classic world.
  --real-time-factor VALUE
                      Slow down or speed up Gazebo sim time. Example: 0.2
  --sim-ready-timeout SEC
                      Wait up to SEC for simulation topics before recording. Default: 300
  --scan-pattern-line-lookup
                      Use precise but slow scan-pattern line lookup.
  --rviz-config PATH  Use a custom RViz config.
  --lidar-pattern FILE
                       Use a LiDAR pattern xacro file name.
  --robot-name NAME   Set the spawned robot name.
  --drive-scenario FILE
                       Start drive_robot.sh with a YAML scenario file.
  --drive-start-delay SEC
                       Wait before starting scripted drive. With --record-bag, the wait is completed before recording. Default: 0.0
  --no-auto-exit      Keep simulation running after a non-loop drive scenario finishes.
  --record-bag        Record default livox and tf topics after simulation readiness checks complete.
  --bag-all-topics    With --record-bag, record all topics instead of the default selected topics.
  --bag-output PATH   Set rosbag output directory or prefix.
  --bag-topics CSV    Record only the given comma-separated topics. Default records selected livox and tf topics.
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
  local pattern_file=""

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

validate_positive_double() {
  local option="$1"
  local value="$2"

  # physics設定へ渡す値は数値として厳密に扱い、Gazebo起動後の不明瞭な失敗を避ける。
  if [[ ! "${value}" =~ ^[+]?[0-9]+([.][0-9]*)?([eE][+-]?[0-9]+)?$ && ! "${value}" =~ ^[+]?[.][0-9]+([eE][+-]?[0-9]+)?$ ]]; then
    echo "${option} must be a positive number: ${value}" >&2
    exit 2
  fi
  if ! awk -v value="${value}" 'BEGIN { exit(value > 0.0 ? 0 : 1) }'; then
    echo "${option} must be greater than 0: ${value}" >&2
    exit 2
  fi
}

validate_non_negative_double() {
  local option="$1"
  local value="$2"

  # 待機時間は0を許容しつつ数値だけを受け入れ、sim時刻待機の計算失敗を起動前に検出する。
  if [[ ! "${value}" =~ ^[+]?[0-9]+([.][0-9]*)?([eE][+-]?[0-9]+)?$ && ! "${value}" =~ ^[+]?[.][0-9]+([eE][+-]?[0-9]+)?$ ]]; then
    echo "${option} must be a non-negative number: ${value}" >&2
    exit 2
  fi
  if ! awk -v value="${value}" 'BEGIN { exit(value >= 0.0 ? 0 : 1) }'; then
    echo "${option} must be greater than or equal to 0: ${value}" >&2
    exit 2
  fi
}

default_world_file() {
  printf '%s/ros2_ws/src/ai_ship_robot_gazebo/worlds/lidar_placement.world' "${SIM_ROOT}"
}

make_real_time_factor_world() {
  local source_world="$1"
  local real_time_factor="$2"
  local max_step_size="0.001"
  local target_world=""
  local update_rate=""

  if [[ ! -f "${source_world}" ]]; then
    echo "World file not found: ${source_world}" >&2
    exit 2
  fi

  target_world="$(mktemp "${TMPDIR:-/tmp}/ai_ship_robot_world_XXXXXX.world")"
  update_rate="$(awk -v factor="${real_time_factor}" -v step="${max_step_size}" 'BEGIN { printf "%.12g", factor / step }')"

  # 既存worldのphysics blockだけを書き換え、モデルや地形定義は元ファイルをそのまま使う。
  if ! awk \
    -v update_rate="${update_rate}" \
    -v max_step_size="${max_step_size}" \
    -v real_time_factor="${real_time_factor}" '
      {
        if ($0 ~ /<real_time_update_rate>[^<]*<\/real_time_update_rate>/) {
          sub(/<real_time_update_rate>[^<]*<\/real_time_update_rate>/, "<real_time_update_rate>" update_rate "</real_time_update_rate>")
          update_seen = 1
        }
        if ($0 ~ /<max_step_size>[^<]*<\/max_step_size>/) {
          sub(/<max_step_size>[^<]*<\/max_step_size>/, "<max_step_size>" max_step_size "</max_step_size>")
          step_seen = 1
        }
        if ($0 ~ /<real_time_factor>[^<]*<\/real_time_factor>/) {
          sub(/<real_time_factor>[^<]*<\/real_time_factor>/, "<real_time_factor>" real_time_factor "</real_time_factor>")
          factor_seen = 1
        }
        print
      }
      END {
        if (!update_seen || !step_seen || !factor_seen) {
          exit 42
        }
      }
    ' "${source_world}" > "${target_world}"; then
    rm -f "${target_world}"
    echo "World file must contain real_time_update_rate, max_step_size, and real_time_factor tags: ${source_world}" >&2
    exit 2
  fi

  TEMP_WORLD_FILE="${target_world}"
  echo "Using Gazebo real_time_factor=${real_time_factor} with max_step_size=${max_step_size}, real_time_update_rate=${update_rate}" >&2
  printf '%s' "${target_world}"
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

append_gazebo_model_path() {
  local model_dir="$1"

  if [[ ! -d "${model_dir}" ]]; then
    return 0
  fi

  # Gazebo Classic が model://ai_ship_robot_description を解決できるよう起動直前の環境にも反映する。
  case ":${GAZEBO_MODEL_PATH:-}:" in
    *":${model_dir}:"*) ;;
    *) export GAZEBO_MODEL_PATH="${model_dir}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}" ;;
  esac
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
    echo "Run bash install/install_third_party.sh && bash sim/install/install_third_party.sh && bash sim/scripts/run_simulation.sh --build or --clean-build." >&2
    return 1
  fi

  source "${setup_file}"
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

start_rosbag_record() {
  local output_path="$1"
  local use_sim_time="$2"
  local record_all_topics="$3"
  shift 3
  local topics=("$@")
  local record_cmd=(ros2 bag record --include-hidden-topics)

  mkdir -p "${ROSBAG_ROOT}"
  if [[ -n "${output_path}" ]]; then
    record_cmd+=(--output "${output_path}")
  fi
  if [[ "${use_sim_time}" == "true" ]]; then
    record_cmd+=(--use-sim-time)
  fi

  # 明示opt-in時だけ全topic記録へ切り替え、既定では必要topicだけに絞る。
  if [[ "${record_all_topics}" == "true" ]]; then
    record_cmd+=(--all)
  else
    record_cmd+=("${topics[@]}")
  fi
  echo "Rosbag output: ${output_path}" >&2
  if [[ "${record_all_topics}" == "true" ]]; then
    echo "Recording rosbag topics: all topics" >&2
  else
    echo "Recording rosbag topics: ${topics[*]}" >&2
  fi
  "${record_cmd[@]}" &
  ROSBAG_PID=$!
}

wait_for_topic_once() {
  local topic="$1"
  local timeout_seconds="$2"
  shift 2
  local echo_args=("$@")
  local start_seconds=${SECONDS}

  echo "Waiting for simulation topic: ${topic}" >&2
  while true; do
    ensure_simulation_process_running

    # topic型の検出前はros2 topic echoが即終了するため、期限までは短いechoを再試行する。
    if timeout 2s ros2 topic echo --once "${echo_args[@]}" "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    if awk -v elapsed="$((SECONDS - start_seconds))" -v timeout="${timeout_seconds}" 'BEGIN { exit(elapsed >= timeout ? 0 : 1) }'; then
      echo "Timed out waiting for simulation topic: ${topic}" >&2
      exit 1
    fi
    sleep 0.2
  done
}

wait_for_topic_subscribers() {
  local topic="$1"
  local min_subscribers="$2"
  local timeout_seconds="$3"
  local start_seconds=${SECONDS}
  local elapsed_seconds=0
  local last_log_seconds=0
  local topic_info=""
  local subscription_count=""

  echo "Waiting for simulation topic subscriber: ${topic} >= ${min_subscribers}" >&2
  while true; do
    ensure_simulation_process_running

    # 走行pluginの購読開始を確認し、シナリオ先頭のcmd_velがGazebo側で捨てられないようにする。
    if topic_info="$(timeout 5s ros2 topic info --no-daemon --spin-time 2 "${topic}" 2>/dev/null)"; then
      subscription_count="$(awk '/Subscription count:/ { print $3; exit }' <<< "${topic_info}")"
      if [[ "${subscription_count}" =~ ^[0-9]+$ && "${subscription_count}" -ge "${min_subscribers}" ]]; then
        return 0
      fi
    fi
    elapsed_seconds=$((SECONDS - start_seconds))
    # direct discoveryの取りこぼしと実subscriber不足を切り分けやすくするため、一定間隔で観測値を出す。
    if [[ "${elapsed_seconds}" -ge $((last_log_seconds + 5)) ]]; then
      echo "Still waiting for simulation topic subscriber: ${topic} observed=${subscription_count:-unknown} required=${min_subscribers}" >&2
      last_log_seconds=${elapsed_seconds}
    fi
    if awk -v elapsed="${elapsed_seconds}" -v timeout="${timeout_seconds}" 'BEGIN { exit(elapsed >= timeout ? 0 : 1) }'; then
      echo "Timed out waiting for simulation topic subscriber: ${topic}" >&2
      exit 1
    fi
    sleep 0.2
  done
}

ensure_no_existing_gazebo_server() {
  local gazebo_master_uri="${GAZEBO_MASTER_URI:-http://localhost:11345}"
  local gzserver_processes=()
  local gzserver_process=""

  if [[ "${gazebo_master_uri}" != "http://localhost:11345" ]]; then
    return 0
  fi

  mapfile -t gzserver_processes < <(pgrep -a -x gzserver 2>/dev/null || true)
  if [[ "${#gzserver_processes[@]}" -eq 0 ]]; then
    return 0
  fi

  # 既定Gazebo masterは1プロセスしかbindできないため、残存serverを起動前に検出して原因を明示する。
  echo "Another gzserver is already running on the default Gazebo master: ${gazebo_master_uri}" >&2
  echo "Stop the existing simulation before starting a new one." >&2
  for gzserver_process in "${gzserver_processes[@]}"; do
    echo "  ${gzserver_process}" >&2
  done
  echo "Example: kill -INT <PID>" >&2
  exit 1
}

ensure_simulation_process_running() {
  local process_status=0

  # readiness待機中にlaunchが落ちた場合は、topic待ちtimeoutではなくlaunchの終了を前面に出す。
  if [[ -n "${SIMULATION_PID}" ]] && ! kill -0 "${SIMULATION_PID}" 2>/dev/null; then
    set +e
    wait "${SIMULATION_PID}"
    process_status=$?
    set -e
    echo "Simulation exited before readiness check completed." >&2
    exit "${process_status}"
  fi
}

wait_for_simulation_ready() {
  # spawn後の走行pluginとLIO-SAM入力topicが揃ってからrecord/driveを開始し、bag先頭の空白を避ける。
  ensure_simulation_process_running
  wait_for_topic_once "/clock" "${SIM_READY_TIMEOUT_SECONDS}" --qos-reliability best_effort
  ensure_simulation_process_running
  wait_for_topic_once "/tf_static" "${SIM_READY_TIMEOUT_SECONDS}" --qos-durability transient_local --qos-reliability reliable
  ensure_simulation_process_running
  wait_for_topic_once "/lidar1/livox/lidar" "${SIM_READY_TIMEOUT_SECONDS}" --qos-reliability reliable
  ensure_simulation_process_running
  wait_for_topic_once "/lidar1/livox/imu" "${SIM_READY_TIMEOUT_SECONDS}" --qos-reliability reliable
  ensure_simulation_process_running
  wait_for_topic_once "/odom" "${SIM_READY_TIMEOUT_SECONDS}"
  ensure_simulation_process_running
  wait_for_topic_subscribers "/cmd_vel" 1 "${SIM_READY_TIMEOUT_SECONDS}"
  echo "Simulation topics are ready." >&2
}

start_scripted_drive() {
  local start_delay="${1:-${DRIVE_START_DELAY}}"
  local drive_cmd=(
    bash "${SIM_ROOT}/scripts/drive_robot.sh"
    --scenario "${DRIVE_SCENARIO}"
    --start-delay "${start_delay}"
  )

  echo "Starting scripted drive scenario: ${DRIVE_SCENARIO}" >&2
  "${drive_cmd[@]}" &
  DRIVE_PID=$!
}

start_custommsg_pointcloud_bridge() {
  # 変換nodeは入力topic末尾に/pointsを付けて出力するため、RViz購読名の親topicを渡す。
  local bridge_cmd=(
    ros2 run ai_ship_robot_slam livox_custommsg_to_pointcloud2_node --ros-args
    -p use_sim_time:=true
    -p input_topics:="['/lidar1/livox/lidar','/lidar2/livox/lidar','/lidar3/livox/lidar','/lidar4/livox/lidar']"
  )

  echo "Starting CustomMsg->PointCloud2 bridge for RViz..." >&2
  "${bridge_cmd[@]}" &
  POINTCLOUD_BRIDGE_PID=$!
}

default_bag_output() {
  # 保存先をworkspace直下へ集約し、開発時の回収場所を固定する。
  printf '%s/sim_%(%Y%m%d_%H%M%S)T' "${ROSBAG_ROOT}" -1
}

collect_child_pids() {
  local parent_pid="$1"
  local child_pid=""

  # ros2 launch配下のGazeboやnodeも停止対象に含め、親だけが残る/子だけが残る状態を避ける。
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

  # 待機中にtimer側が段階的に強制停止し、無制限waitでscript全体が詰まることを防ぐ。
  set +e
  wait "${pid}" 2>/dev/null
  process_status=$?
  kill "${stopper_pid}" 2>/dev/null || true
  wait "${stopper_pid}" 2>/dev/null || true
  set -e
  return "${process_status}"
}

stop_external_process() {
  local pid="$1"
  local label="$2"
  local grace_seconds="${3:-${PROCESS_STOP_GRACE_SECONDS}}"
  local term_seconds="${4:-${PROCESS_STOP_TERM_SECONDS}}"
  local wait_started=${SECONDS}

  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || return 0

  echo "Stopping ${label}..." >&2
  kill -INT "${pid}" 2>/dev/null || true
  while kill -0 "${pid}" 2>/dev/null; do
    if [[ $((SECONDS - wait_started)) -ge "${grace_seconds}" ]]; then
      echo "${label} did not stop after ${grace_seconds}s; sending TERM." >&2
      kill -TERM "${pid}" 2>/dev/null || true
      sleep "${term_seconds}"
      if kill -0 "${pid}" 2>/dev/null; then
        echo "${label} did not stop after TERM; sending KILL." >&2
        kill -KILL "${pid}" 2>/dev/null || true
      fi
      break
    fi
    sleep 0.2
  done
}

stop_gazebo_servers_for_world() {
  local world_path="$1"
  local gzserver_process=""
  local gzserver_pid=""

  [[ -n "${world_path}" ]] || return 0

  # ros2 launch終了後にgzserverだけが孤立する場合があるため、このrunで使ったworldに限定して止める。
  while IFS= read -r gzserver_process; do
    [[ -n "${gzserver_process}" ]] || continue
    if [[ "${gzserver_process}" == *" ${world_path}"* ]]; then
      gzserver_pid="${gzserver_process%% *}"
      stop_external_process "${gzserver_pid}" "gazebo server for ${world_path}" 10 3 || true
    fi
  done < <(pgrep -a -x gzserver 2>/dev/null || true)
}

log_recorded_bag_output() {
  local rosbag_path="${BAG_OUTPUT%/}"
  local rosbag_name=""

  [[ "${RECORD_BAG}" == "true" && -n "${rosbag_path}" ]] || return 0
  [[ -f "${rosbag_path}/metadata.yaml" ]] || return 0

  # 実際にmetadataがflushされたbagだけを再掲し、起動失敗時に未作成bagを保存済み扱いしない。
  rosbag_name="${rosbag_path##*/}"
  echo "Rosbag saved: ${rosbag_name}" >&2
  echo "Rosbag path: ${rosbag_path}" >&2
}

cleanup_background_processes() {
  # 先に自動運転を止め、終了処理中にcmd_vel publishが継続しないようにする。
  stop_background_process "${DRIVE_PID}" "scripted drive" 5 3 || true

  # 可視化専用の補助点群は他processより先に止め、終了中のpublishを抑える。
  stop_background_process "${POINTCLOUD_BRIDGE_PID}" "custommsg pointcloud bridge" 5 3 || true

  # rosbag recordへINTを送り、metadata flushの機会を与えてbag破損を避ける。
  stop_background_process "${ROSBAG_PID}" "rosbag recorder" "${ROSBAG_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  log_recorded_bag_output

  # シナリオ同時起動時はlaunchをbackground管理するため、終了時に明示停止する。
  stop_background_process "${SIMULATION_PID}" "simulation" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
  stop_gazebo_servers_for_world "${WORLD_PATH}"

  # real_time_factor指定時に生成した一時worldは、Gazebo終了後に確実に消す。
  if [[ -n "${TEMP_WORLD_FILE}" && -f "${TEMP_WORLD_FILE}" ]]; then
    rm -f "${TEMP_WORLD_FILE}"
  fi
}

LAUNCH_ARGS=()
WORLD_PATH=""
REAL_TIME_FACTOR=""
SCAN_PATTERN_LINE_LOOKUP=false
BUILD_WORKSPACE=false
CLEAN_BUILD_WORKSPACE=false
LITE_MODE=false
LIDAR_RESOLUTION_MODE="default"
RECORD_BAG=false
RECORD_ALL_BAG_TOPICS=false
BAG_OUTPUT=""
BAG_TOPICS=()
DRIVE_SCENARIO=""
DRIVE_START_DELAY="0.0"
AUTO_EXIT_AFTER_DRIVE=true
DRIVE_OPTION_REQUESTED=false
USE_RVIZ=true
PUBLISH_ODOM_TF=true

trap cleanup_background_processes EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --build)
      BUILD_WORKSPACE=true
      ;;
    --clean-build)
      CLEAN_BUILD_WORKSPACE=true
      BUILD_WORKSPACE=true
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
    --rviz)
      LAUNCH_ARGS+=("use_rviz:=true")
      USE_RVIZ=true
      ;;
    --no-rviz)
      LAUNCH_ARGS+=("use_rviz:=false")
      USE_RVIZ=false
      ;;
    --odom-tf)
      PUBLISH_ODOM_TF=true
      ;;
    --no-odom-tf)
      PUBLISH_ODOM_TF=false
      ;;
    -4|--quarter-resolution)
      LIDAR_RESOLUTION_MODE="quarter"
      ;;
    -2|--half-resolution)
      LIDAR_RESOLUTION_MODE="half"
      ;;
    -1|--full-resolution)
      LIDAR_RESOLUTION_MODE="full"
      ;;
    --world=*)
      WORLD_PATH="${1#*=}"
      ;;
    --world)
      shift
      WORLD_PATH="$(require_value --world "${1:-}")"
      ;;
    --real-time-factor=*)
      REAL_TIME_FACTOR="$(require_value --real-time-factor "${1#*=}")"
      ;;
    --real-time-factor)
      shift
      REAL_TIME_FACTOR="$(require_value --real-time-factor "${1:-}")"
      ;;
    --sim-ready-timeout=*)
      SIM_READY_TIMEOUT_SECONDS="$(require_value --sim-ready-timeout "${1#*=}")"
      ;;
    --sim-ready-timeout)
      shift
      SIM_READY_TIMEOUT_SECONDS="$(require_value --sim-ready-timeout "${1:-}")"
      ;;
    --scan-pattern-line-lookup)
      SCAN_PATTERN_LINE_LOOKUP=true
      ;;
    --rviz-config=*)
      LAUNCH_ARGS+=("rviz_config:=${1#*=}")
      ;;
    --rviz-config)
      shift
      LAUNCH_ARGS+=("rviz_config:=$(require_value --rviz-config "${1:-}")")
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
      ;;
    --robot-name)
      shift
      LAUNCH_ARGS+=("robot_name:=$(require_value --robot-name "${1:-}")")
      ;;
    --drive-scenario=*)
      DRIVE_SCENARIO="$(require_value --drive-scenario "${1#*=}")"
      ;;
    --drive-scenario)
      shift
      DRIVE_SCENARIO="$(require_value --drive-scenario "${1:-}")"
      ;;
    --drive-start-delay=*)
      DRIVE_START_DELAY="$(require_value --drive-start-delay "${1#*=}")"
      DRIVE_OPTION_REQUESTED=true
      ;;
    --drive-start-delay)
      shift
      DRIVE_START_DELAY="$(require_value --drive-start-delay "${1:-}")"
      DRIVE_OPTION_REQUESTED=true
      ;;
    --no-auto-exit)
      AUTO_EXIT_AFTER_DRIVE=false
      ;;
    --record-bag)
      RECORD_BAG=true
      ;;
    --bag-all-topics)
      RECORD_ALL_BAG_TOPICS=true
      ;;
    --bag-output=*)
      BAG_OUTPUT="${1#*=}"
      ;;
    --bag-output)
      shift
      BAG_OUTPUT="$(require_value --bag-output "${1:-}")"
      ;;
    --bag-topics=*)
      mapfile -t BAG_TOPICS < <(parse_csv_topics "${1#*=}")
      ;;
    --bag-topics)
      shift
      mapfile -t BAG_TOPICS < <(parse_csv_topics "$(require_value --bag-topics "${1:-}")")
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

if [[ -z "${DRIVE_SCENARIO}" && "${DRIVE_OPTION_REQUESTED}" == "true" ]]; then
  echo "Drive options require --drive-scenario FILE." >&2
  exit 2
fi

if [[ -n "${DRIVE_SCENARIO}" ]]; then
  validate_non_negative_double --drive-start-delay "${DRIVE_START_DELAY}"
fi

if [[ -n "${DRIVE_SCENARIO}" && ! -f "${DRIVE_SCENARIO}" ]]; then
  echo "Drive scenario file not found: ${DRIVE_SCENARIO}" >&2
  exit 2
fi

# SLAM/localizationと併用する場合は、Gazebo由来のodom TFを止めてTF競合を避けられるようにする。
LAUNCH_ARGS+=("publish_odom_tf:=${PUBLISH_ODOM_TF}")

if [[ -n "${REAL_TIME_FACTOR}" ]]; then
  validate_positive_double --real-time-factor "${REAL_TIME_FACTOR}"
  if [[ -z "${WORLD_PATH}" ]]; then
    WORLD_PATH="$(default_world_file)"
  fi
  WORLD_PATH="$(make_real_time_factor_world "${WORLD_PATH}" "${REAL_TIME_FACTOR}")"
  TEMP_WORLD_FILE="${WORLD_PATH}"
fi

validate_positive_double --sim-ready-timeout "${SIM_READY_TIMEOUT_SECONDS}"

if [[ -n "${WORLD_PATH}" ]]; then
  # world指定は最終的な有効パスだけをlaunchへ渡し、通常worldと一時worldの経路を統一する。
  LAUNCH_ARGS+=("world:=${WORLD_PATH}")
fi

# 既定は高速なindex割り当てにし、必要時だけ旧来のscan pattern逆引きを使う。
LAUNCH_ARGS+=("use_scan_pattern_line_lookup:=${SCAN_PATTERN_LINE_LOOKUP}")

# lite指定時は1/4解像度を既定値にし、個別オプションがあればそちらを優先する。
LAUNCH_ARGS+=("lite:=${LITE_MODE}")
case "${LIDAR_RESOLUTION_MODE}" in
  quarter)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=true")
    ;;
  half)
    LAUNCH_ARGS+=("half_lidar_resolution:=true" "quarter_lidar_resolution:=false")
    ;;
  full)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=false")
    ;;
  default)
    LAUNCH_ARGS+=("half_lidar_resolution:=false" "quarter_lidar_resolution:=false")
    ;;
esac

source_workspace_environment false

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  # simulation単体起動では simulation workspace だけを更新し、clean指定時だけその生成物を再作成する。
  if [[ "${CLEAN_BUILD_WORKSPACE}" == "true" ]]; then
    bash "${SETUP_SIMULATION_SCRIPT}" --clean-build
  else
    bash "${SETUP_SIMULATION_SCRIPT}"
  fi
fi

if [[ ! -f "${SIM_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "Missing sim/ros2_ws/install/setup.bash. Run bash sim/install/setup.sh first." >&2
  exit 1
fi

source_workspace_environment
append_gazebo_model_path "${GAZEBO_MODEL_DIR}"

if [[ "${RECORD_BAG}" == "true" ]]; then
  if [[ -z "${BAG_OUTPUT}" ]]; then
    BAG_OUTPUT="$(default_bag_output)"
  fi
  if [[ "${RECORD_ALL_BAG_TOPICS}" == "false" && "${#BAG_TOPICS[@]}" -eq 0 ]]; then
    BAG_TOPICS=(
      "/lidar1/livox/lidar"
      "/lidar1/livox/imu"
      "/lidar2/livox/lidar"
      "/lidar2/livox/imu"
      "/lidar3/livox/lidar"
      "/lidar3/livox/imu"
      "/lidar4/livox/lidar"
      "/lidar4/livox/imu"
      "/tf"
      "/tf_static"
    )
  fi
fi

if [[ -n "${DRIVE_SCENARIO}" || "${RECORD_BAG}" == "true" ]]; then
  ensure_no_existing_gazebo_server

  # readiness確認やrecord開始を行う場合は、launchをbackground管理して起動完了を待つ。
  ros2 launch ai_ship_robot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}" &
  SIMULATION_PID=$!

  wait_for_simulation_ready

  if [[ "${USE_RVIZ}" == "true" ]]; then
    start_custommsg_pointcloud_bridge
  fi

  if [[ "${RECORD_BAG}" == "true" ]]; then
    # 明示指定があればそのtopicだけを記録し、未指定時は全topicを記録する。
    start_rosbag_record "${BAG_OUTPUT}" true "${RECORD_ALL_BAG_TOPICS}" "${BAG_TOPICS[@]}"
  fi

  if [[ -n "${DRIVE_SCENARIO}" ]]; then
    start_scripted_drive "${DRIVE_START_DELAY}"
  fi

  if [[ -z "${DRIVE_SCENARIO}" ]]; then
    set +e
    wait "${SIMULATION_PID}"
    simulation_status=$?
    set -e
    exit "${simulation_status}"
  fi

  # driveの終了有無を基準に後処理を統一し、loop有無はscenario YAML側へ完全移譲する。
  set +e
  wait "${DRIVE_PID}"
  drive_status=$?
  set -e

  if [[ "${drive_status}" -ne 0 ]]; then
    exit "${drive_status}"
  fi

  if [[ "${AUTO_EXIT_AFTER_DRIVE}" == "true" ]] && kill -0 "${SIMULATION_PID}" 2>/dev/null; then
    echo "Scripted drive finished. Stopping simulation after ${AUTO_STOP_AFTER_DRIVE_SECONDS}s..." >&2
    sleep "${AUTO_STOP_AFTER_DRIVE_SECONDS}"
    stop_background_process "${SIMULATION_PID}" "simulation" "${PROCESS_STOP_GRACE_SECONDS}" "${PROCESS_STOP_TERM_SECONDS}" || true
    exit 0
  fi

  set +e
  wait "${SIMULATION_PID}"
  simulation_status=$?
  set -e
  exit "${simulation_status}"
fi

if [[ "${USE_RVIZ}" == "true" ]]; then
  # plugin側のPointCloud2 publishは無効なため、通常起動でもRViz用変換bridgeを併走させる。
  ros2 launch ai_ship_robot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}" &
  SIMULATION_PID=$!
  start_custommsg_pointcloud_bridge

  set +e
  wait "${SIMULATION_PID}"
  simulation_status=$?
  set -e
  exit "${simulation_status}"
fi

append_gazebo_model_path "${GAZEBO_MODEL_DIR}"
ros2 launch ai_ship_robot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}"
