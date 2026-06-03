#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
export DEBIAN_FRONTEND

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    ${SUDO} rosdep init || true
  fi
  ${SUDO} apt-get update
  rosdep update
  rosdep install --from-paths "${ROS_WS}/src" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
fi

# Deleted files can remain under install/ in a colcon workspace.
rm -rf "${ROS_WS}/build" "${ROS_WS}/install" "${ROS_WS}/log"

colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS}/src" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
