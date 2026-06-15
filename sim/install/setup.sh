#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
ROS_SIM_WS="${SIM_ROOT}/ros2_ws"
ROS_SIM_WS_SRC_DIR="${ROS_SIM_WS}/src"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_WS="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws"
THIRD_PARTY_UNDERLAY_SETUP="${THIRD_PARTY_WS}/install/setup.bash"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
ROSDEP_UPDATED=0

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

write_shell_environment_setup() {
  local bashrc_path="${AI_SHIP_ROBOT_BASHRC:-${HOME}/.bashrc}"
  local env_script_path="${AI_SHIP_ROBOT_ENV_SCRIPT:-${HOME}/.ai_ship_robot_environment.bash}"
  local bashrc_dir=""
  local env_script_dir=""
  local begin_marker="# >>> ai_ship_robot environment >>>"
  local end_marker="# <<< ai_ship_robot environment <<<"
  local filtered_bashrc=""
  local updated_bashrc=""
  local skip_block=0
  local line=""

  bashrc_dir="$(dirname "${bashrc_path}")"
  env_script_dir="$(dirname "${env_script_path}")"
  filtered_bashrc="$(mktemp)"
  updated_bashrc="$(mktemp)"

  # simulation追加後はsystem underlay、runtime、simulationの順に同じ入口から読み込めるようにする。
  mkdir -p "${env_script_dir}"
  cat > "${env_script_path}" <<EOF
# This file is managed by sim/install/setup.sh.
export AI_SHIP_ROBOT_WORKSPACE="${WORKSPACE_ROOT}"
export AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT}"
export ROS_DISTRO="\${ROS_DISTRO:-${ROS_DISTRO}}"
if [ -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]; then
  _ai_ship_robot_source_overlay_if_current() {
    _ai_ship_robot_setup_file="\$1"
    if [ ! -f "\${_ai_ship_robot_setup_file}" ]; then
      return 0
    fi
    if grep -Fq "\${AI_SHIP_ROBOT_WORKSPACE}/third_party/ws" "\${_ai_ship_robot_setup_file}" 2>/dev/null \
      || grep -Fq "\${AI_SHIP_ROBOT_WORKSPACE}/third_party/vendor" "\${_ai_ship_robot_setup_file}" 2>/dev/null \
      || grep -Fq "\${AI_SHIP_ROBOT_WORKSPACE}/third_party_ws" "\${_ai_ship_robot_setup_file}" 2>/dev/null \
      || grep -Fq "\${AI_SHIP_ROBOT_WORKSPACE}/third_party_vendor" "\${_ai_ship_robot_setup_file}" 2>/dev/null; then
      return 0
    fi
    source "\${_ai_ship_robot_setup_file}"
  }
  _ai_ship_robot_had_nounset=0
  if [ "\${-}" != "\${-#*u}" ]; then
    _ai_ship_robot_had_nounset=1
    set +u
  fi
  source "/opt/ros/\${ROS_DISTRO}/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/\${ROS_DISTRO}/third_party_ws/install/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/ros2_ws/install/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/sim/ros2_ws/install/setup.bash"
  if [ "\${_ai_ship_robot_had_nounset}" -eq 1 ]; then
    set -u
  fi
  unset _ai_ship_robot_had_nounset _ai_ship_robot_setup_file
  unset -f _ai_ship_robot_source_overlay_if_current
fi
EOF

  mkdir -p "${bashrc_dir}"
  touch "${bashrc_path}"
  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ "${line}" == "${begin_marker}" ]]; then skip_block=1; continue; fi
    if [[ "${line}" == "${end_marker}" ]]; then skip_block=0; continue; fi
    if [[ "${skip_block}" -eq 0 ]]; then printf '%s\n' "${line}" >> "${filtered_bashrc}"; fi
  done < "${bashrc_path}"

  {
    cat <<EOF
${begin_marker}
if [ -f "${env_script_path}" ]; then
  source "${env_script_path}"
fi
${end_marker}
EOF
    printf '\n'
    if [[ -s "${filtered_bashrc}" ]]; then cat "${filtered_bashrc}"; fi
  } > "${updated_bashrc}"

  mv "${updated_bashrc}" "${bashrc_path}"
  rm -f "${filtered_bashrc}"
  echo "Updated shell environment setup: ${bashrc_path}"
  echo "Updated shell environment script: ${env_script_path}"
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
  require_simulation_underlay_package
  remove_stale_workspace_artifacts
  source_runtime_workspace_if_exists
  install_rosdeps_for_workspace "${ROS_SIM_WS_SRC_DIR}"
  build_project_workspace
}

require_ros2
setup_simulation_workspace
write_shell_environment_setup
