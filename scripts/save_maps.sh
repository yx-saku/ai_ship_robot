#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
THIRD_PARTY_UNDERLAY_SETUP="/opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
SERVICE_NAME="/lio_sam/save_map"
SERVICE_TYPE="lio_sam/srv/SaveMap"
CLOUD_MAP_ROOT="${WORKSPACE_ROOT}/outputs/cloud_map"
GROUND_CANDIDATE_CLOUD_NAME="ground_candidate_cloud.pcd"
GROUND_CANDIDATE_MAP_NAME="ground_candidate_map.pgm"
GROUND_CANDIDATE_PNG_NAME="ground_candidate_map.png"
GROUND_CANDIDATE_YAML_NAME="ground_candidate_map.yaml"
GROUND_COARSE_CLOUD_NAME="ground_coarse_cloud.pcd"
GROUND_COARSE_MAP_NAME="ground_coarse_map.pgm"
GROUND_COARSE_PNG_NAME="ground_coarse_map.png"
GROUND_COARSE_YAML_NAME="ground_coarse_map.yaml"
GROUND_CANDIDATE_CONFIG_PATH="${GROUND_CANDIDATE_CONFIG_PATH:-${WORKSPACE_ROOT}/ros2_ws/src/ai_ship_robot_slam/config/lio_sam_mid360.yaml}"
MAP_DESTINATION=""
SAVE_MAP_REQUEST_RESOLUTION="0.0"
WAIT_TIMEOUT_SECONDS="${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS:-30}"
CALL_TIMEOUT_SECONDS="${SAVE_MAPS_TIMEOUT_SECONDS:-300}"

usage() {
  cat <<'EOF'
Usage: bash <this script> [DESTINATION]

Arguments:
  DESTINATION   Save map outputs to this directory.
                Default: outputs/cloud_map/map_YYYYmmdd_HHMMSS

Options:
  -h, --help    Show this help.

Examples:
  bash <this script>
  bash <this script> outputs/cloud_map/manual_map
EOF
}

source_overlay_if_present() {
  local setup_file="$1"

  if [[ -f "${setup_file}" ]]; then
    # 実行環境差分を吸収するため、存在するoverlayだけを順に読み込む。
    source "${setup_file}"
  fi
}

source_runtime_environment() {
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    exit 1
  fi

  # set -u有効下でもROS setupを安全にsourceできるよう一時的に緩める。
  if [[ "$-" == *u* ]]; then
    had_nounset=1
    set +u
  fi
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_overlay_if_present "${THIRD_PARTY_UNDERLAY_SETUP}"
  source_overlay_if_present "${WORKSPACE_ROOT}/ros2_ws/install/setup.bash"
  source_overlay_if_present "${SIM_ROOT}/ros2_ws/install/setup.bash"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

default_map_output_dir() {
  printf '%s/map_%s' "${CLOUD_MAP_ROOT}" "$(date +%Y%m%d_%H%M%S)"
}

resolve_map_destination() {
  local destination="${MAP_DESTINATION}"

  if [[ -z "${destination}" ]]; then
    destination="$(default_map_output_dir)"
  fi
  if [[ "${destination}" != /* ]]; then
    destination="${WORKSPACE_ROOT}/${destination}"
  fi
  printf '%s' "${destination}"
}

wait_for_service() {
  local started_seconds=${SECONDS}
  local elapsed_seconds=0
  local last_log_seconds=0
  local service_list=""

  echo "Waiting for LIO-SAM save_map service ${SERVICE_NAME}..." >&2
  while true; do
    service_list="$(timeout 5s ros2 service list --no-daemon 2>/dev/null || true)"
    if grep -Fxq "${SERVICE_NAME}" <<< "${service_list}"; then
      return 0
    fi

    elapsed_seconds=$((SECONDS - started_seconds))
    if [[ "${WAIT_TIMEOUT_SECONDS}" != "0" &&
          "${WAIT_TIMEOUT_SECONDS}" != "0.0" &&
          "${elapsed_seconds}" -ge "${WAIT_TIMEOUT_SECONDS}" ]]; then
      echo "Timed out waiting for LIO-SAM save_map service after ${elapsed_seconds}s." >&2
      return 1
    fi

    # 長時間待機時も進捗を出し、停止ではなく待機中だと判別できるようにする。
    if (( SECONDS - last_log_seconds >= 5 )); then
      echo "Still waiting for LIO-SAM save_map service ${SERVICE_NAME}." >&2
      last_log_seconds=${SECONDS}
    fi
    sleep 0.5
  done
}

generate_ground_candidate_outputs() {
  local map_dir="$1"
  local ground_cloud="${map_dir}/${GROUND_CANDIDATE_CLOUD_NAME}"
  local ground_map="${map_dir}/${GROUND_CANDIDATE_MAP_NAME}"
  local ground_png="${map_dir}/${GROUND_CANDIDATE_PNG_NAME}"
  local ground_yaml="${map_dir}/${GROUND_CANDIDATE_YAML_NAME}"
  local coarse_cloud="${map_dir}/${GROUND_COARSE_CLOUD_NAME}"
  local coarse_map="${map_dir}/${GROUND_COARSE_MAP_NAME}"
  local coarse_png="${map_dir}/${GROUND_COARSE_PNG_NAME}"
  local coarse_yaml="${map_dir}/${GROUND_COARSE_YAML_NAME}"
  local generator_output=""
  local generator_status=0
  local generator_cmd=(
    ros2 run ai_ship_robot_slam ground_candidate_map_generator
    --map-dir "${map_dir}"
  )

  if [[ -f "${GROUND_CANDIDATE_CONFIG_PATH}" ]]; then
    generator_cmd+=(--config "${GROUND_CANDIDATE_CONFIG_PATH}")
  fi

  echo "Starting ground candidate generation: map_dir=${map_dir} config=${GROUND_CANDIDATE_CONFIG_PATH}" >&2
  set +e
  generator_output="$("${generator_cmd[@]}" 2>&1)"
  generator_status=$?
  set -e

  if [[ "${generator_status}" -ne 0 ]]; then
    echo "Ground candidate generation failed: ${generator_output//$'\n'/ }" >&2
    return 1
  fi

  # save_map成果物として扱えるよう、生成ログと主要ファイルパスを保存処理のログへ明示する。
  echo "Ground candidate generation completed: ${generator_output//$'\n'/ }" >&2
  echo "ground_candidate_cloud=${ground_cloud}" >&2
  echo "ground_candidate_map=${ground_map}" >&2
  echo "ground_candidate_png=${ground_png}" >&2
  echo "ground_candidate_yaml=${ground_yaml}" >&2
  echo "ground_coarse_cloud=${coarse_cloud}" >&2
  echo "ground_coarse_map=${coarse_map}" >&2
  echo "ground_coarse_png=${coarse_png}" >&2
  echo "ground_coarse_yaml=${coarse_yaml}" >&2
}

request_map_save() {
  local destination="$(resolve_map_destination)"
  local localization_pcd="${destination}/localization_map.pcd"
  local seed_ring_pcd="${destination}/trajectory_seed_ring_cloud.pcd"
  local service_request="{resolution: ${SAVE_MAP_REQUEST_RESOLUTION}, destination: \"${destination}\"}"
  local call_output=""
  local call_status=0
  local call_cmd=(ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" "${service_request}")

  echo "Starting map outputs save request: destination=${destination}" >&2
  set +e
  if [[ "${CALL_TIMEOUT_SECONDS}" == "0" || "${CALL_TIMEOUT_SECONDS}" == "0.0" ]]; then
    call_output="$("${call_cmd[@]}" 2>&1)"
    call_status=$?
  else
    call_output="$(timeout "${CALL_TIMEOUT_SECONDS}s" "${call_cmd[@]}" 2>&1)"
    call_status=$?
  fi
  set -e

  # service callは終了コード0でもsuccess=falseを返し得るため本文まで検査する。
  if [[ "${call_status}" -eq 0 &&
        ( "${call_output}" == *"success=True"* ||
          "${call_output}" == *"success: true"* ||
          "${call_output}" == *"success: True"* ) ]]; then
    echo "Map outputs save completed: ${call_output//$'\n'/ }" >&2
    echo "output_dir=${destination}" >&2
    echo "localization_map=${localization_pcd}" >&2
    echo "trajectory_seed_ring_cloud=${seed_ring_pcd}" >&2
    if ! generate_ground_candidate_outputs "${destination}"; then
      return 1
    fi
    return 0
  fi

  echo "Map outputs save failed: ${call_output//$'\n'/ }" >&2
  return 1
}

parse_args() {
  if [[ $# -eq 0 ]]; then
    return 0
  fi
  if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
    exit 0
  fi
  if [[ "$1" == --* ]]; then
    echo "Unknown option: $1" >&2
    usage >&2
    exit 2
  fi
  if [[ $# -gt 1 ]]; then
    echo "Only one destination path can be specified." >&2
    usage >&2
    exit 2
  fi
  MAP_DESTINATION="$1"
}

main() {
  parse_args "$@"
  source_runtime_environment
  wait_for_service
  request_map_save
}

main "$@"
