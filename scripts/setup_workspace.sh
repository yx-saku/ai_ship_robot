#!/usr/bin/env bash
# 環境を操作するshellでsourceし、ROS 2とビルド済みoverlayを読み込む。
# 依存インストール、外部repo取得、workspace buildはinstall_environment.shで行う。

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "setup_workspace.sh is intended to be sourced:" >&2
  echo "  source scripts/setup_workspace.sh" >&2
  exit 0
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
THIRD_PARTY_WS="${WORKSPACE_ROOT}/third_party_ws"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
  return 1
fi

# ROS setup scriptはnounset-safeではないため、呼び出し元のnounset状態を退避してから読み込む。
_setup_workspace_had_nounset=0
if [[ "$-" == *u* ]]; then
  _setup_workspace_had_nounset=1
  set +u
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 外部workspaceを先に重ね、プロジェクト側がLivox関連packageを参照できるようにする。
if [[ -f "${THIRD_PARTY_WS}/install/setup.bash" ]]; then
  source "${THIRD_PARTY_WS}/install/setup.bash"
fi

if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
  source "${ROS_WS}/install/setup.bash"
fi

if [[ "${_setup_workspace_had_nounset}" -eq 1 ]]; then
  set -u
fi
unset _setup_workspace_had_nounset
