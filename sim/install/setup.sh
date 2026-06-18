#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
ROS_SIM_WS="${SIM_ROOT}/ros2_ws"
ROS_SIM_WS_SRC_DIR="${ROS_SIM_WS}/src"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
THIRD_PARTY_WS="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws"
THIRD_PARTY_UNDERLAY_SETUP="${THIRD_PARTY_WS}/install/setup.bash"

source "${SCRIPT_DIR}/../../install/shell_environment.sh"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
ROSDEP_UPDATED=0
SIM_INSTALL_THIRD_PARTY_SCRIPT="${SIM_ROOT}/install/install_third_party.sh"

usage() {
  cat <<'EOF'
Usage: bash sim/install/setup.sh

simulation向け workspace セットアップを実行します。
- /opt/ai_ship_robot の third_party underlay 読み込み
- rosdep / sim/ros2_ws build
- shell 自動読み込み設定更新

事前に以下を実行してください。
- bash install/install.sh
- bash install/install_third_party.sh
- bash sim/install/install.sh
- bash sim/install/install_third_party.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

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
    echo "Run bash install/install_third_party.sh && bash sim/install/install_third_party.sh first." >&2
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
  # simulation pluginを含むsystem underlayを先に読み込み、sim/ros2_wsの依存先にする。
  set +u
  source "${THIRD_PARTY_UNDERLAY_SETUP}"
  set -u
}

require_simulation_underlay_package() {
  if ! ros2 pkg prefix ros2_livox_simulation >/dev/null 2>&1; then
    echo "Missing ros2_livox_simulation in third-party underlay." >&2
    echo "Run bash sim/install/install_third_party.sh first." >&2
    exit 1
  fi
}

ensure_simulation_underlay_patch_current() {
  local plugin_source="${THIRD_PARTY_WS}/src/ros2_livox_simulation/src/livox_points_plugin.cpp"

  [[ -f "${plugin_source}" ]] || return 0
  if grep -Fq "enable_pointcloud2" "${plugin_source}"; then
    return 0
  fi

  # PointCloud2既定無効化を反映するため、古いplugin sourceを検出したらunderlayを更新する。
  echo "Updating simulation third-party underlay patches..."
  bash "${SIM_INSTALL_THIRD_PARTY_SCRIPT}"
}

source_runtime_workspace_if_exists() {
  if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
    return 0
  fi

  # runtime workspaceが既にbuild済みなら重ね、simulation側から共通packageを参照できるようにする。
  set +u
  source "${ROS_WS}/install/setup.bash"
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
  local stale_runtime=0
  local stale_simulation=0

  if setup_file_has_stale_third_party_path "${ROS_WS}/install/setup.bash"; then stale_runtime=1; fi
  if setup_file_has_stale_third_party_path "${ROS_SIM_WS}/install/setup.bash"; then stale_simulation=1; fi
  if [[ "${stale_runtime}" -eq 0 && "${stale_simulation}" -eq 0 ]]; then return 0; fi

  # 旧workspace内third_partyの絶対パスを含む生成物は、新underlayへ切替後のsource順を壊すため再生成する。
  echo "Remove stale workspace artifacts that reference old third_party paths."
  if [[ "${stale_runtime}" -eq 1 ]]; then rm -rf "${ROS_WS}/build" "${ROS_WS}/install" "${ROS_WS}/log"; fi
  if [[ "${stale_simulation}" -eq 1 ]]; then rm -rf "${ROS_SIM_WS}/build" "${ROS_SIM_WS}/install" "${ROS_SIM_WS}/log"; fi
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

  # rosdep cacheが十分新しい場合は更新を省略し、simulation追加時のネットワーク待ちを減らす。
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

  # simulation workspace依存の解決直前だけrosdep indexを更新する。
  rosdep update
  mkdir -p "${HOME}/.ros/rosdep"
  date +%s > "${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  ROSDEP_UPDATED=1
}

install_rosdeps_for_workspace() {
  local workspace_src="$1"

  if [[ ! -d "${workspace_src}" ]] || ! command -v rosdep >/dev/null 2>&1; then return 0; fi

  # simulation overlay追加後にworkspace依存を再確認し、launch側の不足を見落とさないようにする。
  ensure_rosdep_updated
  rosdep install --from-paths "${workspace_src}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

build_project_workspace() {
  if [[ ! -d "${ROS_SIM_WS_SRC_DIR}" ]]; then
    echo "No simulation workspace sources found under ${ROS_SIM_WS_SRC_DIR}" >&2
    return 1
  fi

  # simulation workspaceは専用の sim/ros2_ws だけをbuildし、本番導線と成果物を分離する。
  colcon --log-base "${ROS_SIM_WS}/log" build --base-paths "${ROS_SIM_WS_SRC_DIR}" --build-base "${ROS_SIM_WS}/build" --install-base "${ROS_SIM_WS}/install" --symlink-install
}

setup_simulation_workspace() {
  source_ros2
  require_third_party_underlay
  source_third_party_underlay
  ensure_simulation_underlay_patch_current
  require_simulation_underlay_package
  remove_stale_workspace_artifacts
  source_runtime_workspace_if_exists
  install_rosdeps_for_workspace "${ROS_SIM_WS_SRC_DIR}"
  build_project_workspace
}

require_ros2
setup_simulation_workspace
write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"
