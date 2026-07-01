#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
ROS_WS_SRC_DIR="${ROS_WS}/src"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
THIRD_PARTY_WS="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws"
THIRD_PARTY_UNDERLAY_SETUP="${THIRD_PARTY_WS}/install/setup.bash"

source "${SCRIPT_DIR}/shell_environment.sh"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
ROSDEP_UPDATED=0

usage() {
  cat <<'EOF'
Usage: bash install/setup.sh [--clean-build]

本番向け workspace セットアップを実行します。
- /opt/ai_ship_robot の third_party underlay 読み込み
- rosdep / ros2_ws build
- --clean-build 指定時は ros2_ws/build install log を削除してから再 build
- shell 自動読み込み設定更新

事前に `bash install/install.sh` と `bash install/install_third_party.sh` を実行してください。
EOF
}

CLEAN_BUILD=false

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)
        usage
        exit 0
        ;;
      --clean-build)
        CLEAN_BUILD=true
        ;;
      *)
        echo "Unknown option: $1" >&2
        echo "Run with --help to see available options." >&2
        exit 2
        ;;
    esac
    shift
  done
}

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required when running as a non-root user." >&2
    exit 1
  fi
  SUDO="sudo"
fi

require_ros2() {
  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Run bash install/install.sh first." >&2
    exit 1
  fi
}

require_third_party_underlay() {
  if [[ ! -f "${THIRD_PARTY_UNDERLAY_SETUP}" ]]; then
    echo "Missing third-party underlay: ${THIRD_PARTY_UNDERLAY_SETUP}" >&2
    echo "Run bash install/install_third_party.sh first." >&2
    exit 1
  fi
}

source_ros2() {
  # ROS setup scriptはnounset-safeではないため、読み込み中だけnounsetを解除する。
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

source_third_party_underlay() {
  # system側で固定導入したthird_party underlayを、プロジェクトworkspaceの依存先として読み込む。
  set +u
  source "${THIRD_PARTY_UNDERLAY_SETUP}"
  set -u
}

setup_file_has_stale_third_party_path() {
  local setup_file="$1"

  [[ -f "${setup_file}" ]] || return 1
  grep -Fq "${WORKSPACE_ROOT}/third_party/ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party/vendor" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"
}

remove_stale_workspace_artifacts() {
  if ! setup_file_has_stale_third_party_path "${ROS_WS}/install/setup.bash"; then
    return 0
  fi

  # 旧workspace内third_partyの絶対パスを含む生成物は、新underlayへ切替後のsource順を壊すため再生成する。
  echo "Remove stale ros2_ws artifacts that reference old third_party paths."
  rm -rf "${ROS_WS}/build" "${ROS_WS}/install" "${ROS_WS}/log"
}

remove_runtime_workspace_artifacts_for_clean_build() {
  # 明示的なクリーンビルド要求では、runtime workspace の生成物だけを削除して再生成する。
  echo "Remove ros2_ws artifacts for clean build."
  rm -rf "${ROS_WS}/build" "${ROS_WS}/install" "${ROS_WS}/log"
}

ensure_rosdep_initialized() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    ${SUDO} rosdep init || true
  fi
}

rosdep_cache_fresh() {
  local stamp_file="${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  local cache_dir="${HOME}/.ros/rosdep/sources.cache"
  local updated_at=""
  local now=""

  if [[ ! "${ROSDEP_UPDATE_MAX_AGE_SECONDS}" =~ ^[0-9]+$ || "${ROSDEP_UPDATE_MAX_AGE_SECONDS}" -eq 0 ]]; then return 1; fi
  if [[ ! -f "${stamp_file}" || ! -d "${cache_dir}" ]]; then return 1; fi
  updated_at="$(<"${stamp_file}")"
  if [[ ! "${updated_at}" =~ ^[0-9]+$ ]]; then return 1; fi

  # rosdep cacheが十分新しい場合は更新を省略し、再作成コンテナでのネットワーク待ちを減らす。
  now="$(date +%s)"
  (( now - updated_at < ROSDEP_UPDATE_MAX_AGE_SECONDS ))
}

ensure_rosdep_updated() {
  if ! command -v rosdep >/dev/null 2>&1; then return 0; fi
  ensure_rosdep_initialized
  if [[ "${ROSDEP_UPDATED}" -eq 1 ]]; then return 0; fi
  if rosdep_cache_fresh; then
    echo "Skip rosdep update because cache is fresh. Set ROSDEP_UPDATE_MAX_AGE_SECONDS=0 to force update."
    ROSDEP_UPDATED=1
    return 0
  fi

  # project workspace依存の解決直前だけrosdep indexを更新し、不要なネットワーク待ちを避ける。
  rosdep update
  mkdir -p "${HOME}/.ros/rosdep"
  date +%s > "${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  ROSDEP_UPDATED=1
}

install_rosdeps_for_workspace() {
  local workspace_src="$1"

  if [[ ! -d "${workspace_src}" ]] || ! command -v rosdep >/dev/null 2>&1; then return 0; fi

  # project workspaceの依存はthird_party underlayを読み込んだ後に解決し、find_package失敗を避ける。
  ensure_rosdep_updated
  rosdep install --from-paths "${workspace_src}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

build_project_workspace() {
  if [[ ! -d "${ROS_WS_SRC_DIR}" ]]; then
    echo "No project workspace sources found under ${ROS_WS_SRC_DIR}" >&2
    return 1
  fi

  # project workspaceは開発対象だけをsymlink-installでbuildし、system underlayと責務を分ける。
  colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS_SRC_DIR}" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
}

setup_runtime_workspace() {
  source_ros2
  require_third_party_underlay
  source_third_party_underlay
  if [[ "${CLEAN_BUILD}" == "true" ]]; then
    remove_runtime_workspace_artifacts_for_clean_build
  fi
  remove_stale_workspace_artifacts
  install_rosdeps_for_workspace "${ROS_WS_SRC_DIR}"
  build_project_workspace
}

parse_args "$@"
require_ros2
setup_runtime_workspace
write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"
