#!/usr/bin/env bash

write_shell_environment_setup() {
  local workspace_root="$1"
  local ros_distro="$2"
  local bashrc_path="${HOME}/.bashrc"
  local bashrc_dir=""
  local filtered_bashrc=""
  local updated_bashrc=""
  local begin_marker="# >>> ai_ship_robot environment >>>"
  local end_marker="# <<< ai_ship_robot environment <<<"
  local legacy_env_script_path=""
  local legacy_source_line=""
  local legacy_source_line_unquoted=""
  local skip_block=0
  local line=""

  bashrc_dir="$(dirname "${bashrc_path}")"
  legacy_env_script_path="${bashrc_dir}/.ai_ship_robot_environment.bash"
  legacy_source_line="source \"${legacy_env_script_path}\""
  legacy_source_line_unquoted="source ${legacy_env_script_path}"
  filtered_bashrc="$(mktemp)"
  updated_bashrc="$(mktemp)"

  # 旧env script導線と管理blockを除去し、現在のbashrcへ単一の入口だけを残す。
  mkdir -p "${bashrc_dir}"
  touch "${bashrc_path}"
  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ "${line}" == "${begin_marker}" ]]; then skip_block=1; continue; fi
    if [[ "${line}" == "${end_marker}" ]]; then skip_block=0; continue; fi
    if [[ "${line}" == "${legacy_source_line}" || "${line}" == "${legacy_source_line_unquoted}" ]]; then continue; fi
    if [[ "${skip_block}" -eq 0 ]]; then printf '%s\n' "${line}" >> "${filtered_bashrc}"; fi
  done < "${bashrc_path}"

  # shell起動時は存在するoverlayだけを順に重ね、未build状態でも壊れない環境入口にする。
  cat > "${updated_bashrc}" <<EOF
${begin_marker}
export AI_SHIP_ROBOT_WORKSPACE="${workspace_root}"
export ROS_DISTRO="\${ROS_DISTRO:-${ros_distro}}"
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
  # setup.bash群はnounsetと相性が悪いため、source中だけ一時的に緩める。
  _ai_ship_robot_had_nounset=0
  if [ "\${-}" != "\${-#*u}" ]; then
    _ai_ship_robot_had_nounset=1
    set +u
  fi
  source "/opt/ros/\${ROS_DISTRO}/setup.bash"
  _ai_ship_robot_source_overlay_if_current "/opt/ai_ship_robot/ros_underlay/\${ROS_DISTRO}/third_party_ws/install/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/ros2_ws/install/setup.bash"
  _ai_ship_robot_source_overlay_if_current "\${AI_SHIP_ROBOT_WORKSPACE}/sim/ros2_ws/install/setup.bash"
  if [ "\${_ai_ship_robot_had_nounset}" -eq 1 ]; then
    set -u
  fi
  unset _ai_ship_robot_had_nounset _ai_ship_robot_setup_file
  unset -f _ai_ship_robot_source_overlay_if_current
fi
${end_marker}

EOF

  if [[ -s "${filtered_bashrc}" ]]; then cat "${filtered_bashrc}" >> "${updated_bashrc}"; fi
  mv "${updated_bashrc}" "${bashrc_path}"
  rm -f "${filtered_bashrc}"
  echo "Updated shell environment setup: ${bashrc_path}"
}
