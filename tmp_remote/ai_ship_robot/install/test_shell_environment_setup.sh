#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

assert_contains() {
  local file_path="$1"
  local expected="$2"

  # setupスクリプトのsourceと呼び出し契約を固定し、共通化の崩れを静的に検出する。
  if ! grep -Fq -- "${expected}" "${file_path}"; then
    echo "Expected line not found in ${file_path}: ${expected}" >&2
    exit 1
  fi
}

assert_not_contains() {
  local file_path="$1"
  local unexpected="$2"

  # bashrcの書き込み先はHOME固定にし、古いoverride変数が戻っていないことを確認する。
  if grep -Fq -- "${unexpected}" "${file_path}"; then
    echo "Unexpected line found in ${file_path}: ${unexpected}" >&2
    exit 1
  fi
}

assert_line_count() {
  local file_path="$1"
  local expected_line="$2"
  local expected_count="$3"
  local actual_count=""

  # 管理blockは再実行しても1つだけ残り、ユーザーのbashrcを重複汚染しないことを検査する。
  actual_count="$(grep -Fxc -- "${expected_line}" "${file_path}" || true)"
  if [[ "${actual_count}" != "${expected_count}" ]]; then
    echo "Unexpected count for ${expected_line} in ${file_path}: ${actual_count}" >&2
    exit 1
  fi
}

assert_order() {
  local file_path="$1"
  local earlier="$2"
  local later="$3"
  local earlier_line=""
  local later_line=""

  # devcontainer初回ターミナル対策では、install実行前にユーザー切替が済んでいることが重要になる。
  earlier_line="$(grep -Fnm 1 -- "${earlier}" "${file_path}" | cut -d: -f1 || true)"
  later_line="$(grep -Fnm 1 -- "${later}" "${file_path}" | cut -d: -f1 || true)"
  if [[ -z "${earlier_line}" || -z "${later_line}" || "${earlier_line}" -ge "${later_line}" ]]; then
    echo "Unexpected order in ${file_path}: ${earlier} must appear before ${later}" >&2
    exit 1
  fi
}

test_static_contracts() {
  # runtime/simulation のinstallとsetupで同じ環境書き込み関数を呼び出す。
  assert_contains \
    "${WORKSPACE_ROOT}/install/install.sh" \
    'source "${SCRIPT_DIR}/shell_environment.sh"'
  assert_contains \
    "${WORKSPACE_ROOT}/install/install.sh" \
    'write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"'
  assert_contains \
    "${WORKSPACE_ROOT}/install/setup.sh" \
    'source "${SCRIPT_DIR}/shell_environment.sh"'
  assert_contains \
    "${WORKSPACE_ROOT}/install/setup.sh" \
    'write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"'
  assert_contains \
    "${WORKSPACE_ROOT}/sim/install/setup.sh" \
    'source "${SCRIPT_DIR}/../../install/shell_environment.sh"'
  assert_contains \
    "${WORKSPACE_ROOT}/sim/install/setup.sh" \
    'write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"'
  assert_contains \
    "${WORKSPACE_ROOT}/sim/install/install.sh" \
    'source "${SCRIPT_DIR}/../../install/shell_environment.sh"'
  assert_contains \
    "${WORKSPACE_ROOT}/sim/install/install.sh" \
    'write_shell_environment_setup "${WORKSPACE_ROOT}" "${ROS_DISTRO}"'

  # devcontainerではremoteUserのHOMEへbashrcを書くため、install script実行前にrosユーザーへ切り替える。
  assert_order \
    "${WORKSPACE_ROOT}/.devcontainer/Dockerfile" \
    'USER ${USERNAME}' \
    'bash /workspaces/ai_ship_robot/install/install.sh'
  assert_order \
    "${WORKSPACE_ROOT}/.devcontainer/Dockerfile" \
    'USER ${USERNAME}' \
    'bash /workspaces/ai_ship_robot/sim/install/install.sh'
  assert_not_contains \
    "${WORKSPACE_ROOT}/.devcontainer/Dockerfile" \
    'chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}/.ros'

  # bashrc pathはユーザー実行前提でHOME固定にし、環境変数による差し替え導線を持たない。
  assert_contains \
    "${WORKSPACE_ROOT}/install/shell_environment.sh" \
    'local bashrc_path="${HOME}/.bashrc"'
  assert_not_contains "${WORKSPACE_ROOT}/install/shell_environment.sh" "AI_SHIP_ROBOT_BASHRC"
  assert_not_contains "${WORKSPACE_ROOT}/install/shell_environment.sh" "AI_SHIP_ROBOT_ENV_SCRIPT"
}

test_bashrc_rewrite_idempotent() {
  local temp_home=""
  local bashrc_path=""

  temp_home="$(mktemp -d)"
  bashrc_path="${temp_home}/.bashrc"
  trap 'rm -rf "${temp_home:-}"' RETURN

  # 旧env script導線、旧管理block、ユーザー記述を混ぜ、再生成後の保存対象を明確にする。
  cat > "${bashrc_path}" <<EOF
source "/tmp/keep_me.bash"
# >>> ai_ship_robot environment >>>
export AI_SHIP_ROBOT_WORKSPACE="/old/workspace"
# <<< ai_ship_robot environment <<<
source "${temp_home}/.ai_ship_robot_environment.bash"
export USER_DEFINED_VALUE=1
EOF

  HOME="${temp_home}" bash -c \
    'source "${0}/install/shell_environment.sh"; write_shell_environment_setup "/workspace/test" "humble"; write_shell_environment_setup "/workspace/test" "humble"' \
    "${WORKSPACE_ROOT}"

  # 冪等実行後も管理blockは1つだけで、ユーザー記述と新しいoverlay入口だけが残る。
  assert_line_count "${bashrc_path}" "# >>> ai_ship_robot environment >>>" "1"
  assert_line_count "${bashrc_path}" "# <<< ai_ship_robot environment <<<" "1"
  assert_contains "${bashrc_path}" 'export AI_SHIP_ROBOT_WORKSPACE="/workspace/test"'
  assert_not_contains "${bashrc_path}" 'OPT_ROOT'
  assert_contains "${bashrc_path}" '/opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash'
  assert_contains "${bashrc_path}" 'export USER_DEFINED_VALUE=1'
  assert_contains "${bashrc_path}" 'source "/tmp/keep_me.bash"'
  assert_not_contains "${bashrc_path}" 'export AI_SHIP_ROBOT_WORKSPACE="/old/workspace"'
  assert_not_contains "${bashrc_path}" '.ai_ship_robot_environment.bash'
}

main() {
  test_static_contracts
  test_bashrc_rewrite_idempotent
}

main "$@"
