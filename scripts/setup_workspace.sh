#!/usr/bin/env bash
# Source this file when entering the container to load ROS 2 and workspace overlays.
# Package installation and colcon builds are handled by bootstrap_workspace.sh.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "setup_workspace.sh is intended to be sourced:" >&2
  echo "  source scripts/setup_workspace.sh" >&2
  exit 0
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
  return 1
fi

# ROS setup scripts are not nounset-safe. Preserve the caller's nounset state.
_setup_workspace_had_nounset=0
if [[ "$-" == *u* ]]; then
  _setup_workspace_had_nounset=1
  set +u
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
  source "${ROS_WS}/install/setup.bash"
fi

if [[ "${_setup_workspace_had_nounset}" -eq 1 ]]; then
  set -u
fi
unset _setup_workspace_had_nounset
