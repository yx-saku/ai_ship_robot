#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

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

  # source処理本体を独立ファイルへ置き、bashrc側の管理ブロックは薄い読み込み口だけにする。
  mkdir -p "${env_script_dir}"
  cat > "${env_script_path}" <<EOF
# This file is managed by scripts/install/shell_environment.sh.
export AI_SHIP_ROBOT_WORKSPACE="${WORKSPACE_ROOT}"
export ROS_DISTRO="\${ROS_DISTRO:-${ROS_DISTRO}}"
if [ -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ] && [ -f "/opt/ros/\${ROS_DISTRO}/local_setup.bash" ]; then
  _ai_ship_robot_source_overlay_if_current() {
    _ai_ship_robot_setup_file="\$1"

    if [ ! -f "\${_ai_ship_robot_setup_file}" ]; then
      return 0
    fi

    if grep -Fq "\${AI_SHIP_ROBOT_WORKSPACE}/third_party_ws" "\${_ai_ship_robot_setup_file}" 2>/dev/null \
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
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/third_party/ws/install/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/ros2_ws/install/setup.bash"
  if [ "\${_ai_ship_robot_had_nounset}" -eq 1 ]; then
    set -u
  fi
  unset _ai_ship_robot_had_nounset _ai_ship_robot_setup_file
  unset -f _ai_ship_robot_source_overlay_if_current
fi
EOF

  # 再実行時に設定が重複しないよう、前回の管理ブロックだけを除去してから先頭へ最新内容を置く。
  mkdir -p "${bashrc_dir}"
  touch "${bashrc_path}"
  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ "${line}" == "${begin_marker}" ]]; then
      skip_block=1
      continue
    fi

    if [[ "${line}" == "${end_marker}" ]]; then
      skip_block=0
      continue
    fi

    if [[ "${skip_block}" -eq 0 ]]; then
      printf '%s\n' "${line}" >> "${filtered_bashrc}"
    fi
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
    if [[ -s "${filtered_bashrc}" ]]; then
      cat "${filtered_bashrc}"
    fi
  } > "${updated_bashrc}"

  mv "${updated_bashrc}" "${bashrc_path}"
  rm -f "${filtered_bashrc}"
  echo "Updated shell environment setup: ${bashrc_path}"
  echo "Updated shell environment script: ${env_script_path}"
}

write_shell_environment_setup
