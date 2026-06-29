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
MAP_DESTINATION="${MAP_DESTINATION:-}"
MAP_RESOLUTION="${MAP_RESOLUTION:-0.10}"
WAIT_TIMEOUT_SECONDS="${WAIT_FOR_MAP_SAVE_SERVICE_TIMEOUT_SECONDS:-30}"
CALL_TIMEOUT_SECONDS="${SAVE_MAPS_TIMEOUT_SECONDS:-300}"

usage() {
  cat <<'EOF'
Usage: bash <this script> [OPTIONS]

Options:
  --destination PATH   Save map outputs to PATH. Default: outputs/cloud_map/map_YYYYmmdd_HHMMSS
  --resolution N       Downsample resolution passed to /lio_sam/save_map. Default: 0.10
  --wait-timeout SEC   Max seconds to wait for /lio_sam/save_map service. Default: 30
                       Use 0 to wait without timeout.
  --call-timeout SEC   Max seconds to wait for the save request itself. Default: 300
                       Use 0 to wait without timeout.
  -h, --help           Show this help.

Examples:
  bash <this script>
  bash <this script> --destination outputs/cloud_map/manual_map --resolution 0.10
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

request_map_save() {
  local destination="$(resolve_map_destination)"
  local localization_pcd="${destination}/localization_map.pcd"
  local elevation_manifest="${destination}/elevation_manifest.yaml"
  local elevation_csv="${destination}/global_elevation_map.csv"
  local service_request="{resolution: ${MAP_RESOLUTION}, destination: \"${destination}\"}"
  local call_output=""
  local call_status=0
  local call_cmd=(ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" "${service_request}")

  echo "Starting map outputs save request: destination=${destination} resolution=${MAP_RESOLUTION}" >&2
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
    echo "elevation_manifest=${elevation_manifest}" >&2
    echo "global_elevation_csv=${elevation_csv}" >&2
    return 0
  fi

  echo "Map outputs save failed: ${call_output//$'\n'/ }" >&2
  return 1
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --wait-timeout)
        WAIT_TIMEOUT_SECONDS="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      --destination)
        MAP_DESTINATION="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      --destination=*)
        MAP_DESTINATION="${1#*=}"
        shift
        ;;
      --resolution)
        MAP_RESOLUTION="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      --resolution=*)
        MAP_RESOLUTION="${1#*=}"
        shift
        ;;
      --call-timeout)
        CALL_TIMEOUT_SECONDS="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

main() {
  parse_args "$@"
  source_runtime_environment
  wait_for_service
  request_map_save
}

main "$@"
