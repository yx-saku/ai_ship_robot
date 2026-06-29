#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${WORKSPACE_ROOT}/sim"
THIRD_PARTY_UNDERLAY_SETUP="/opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws/install/setup.bash"
CLOUD_MAP_ROOT="${WORKSPACE_ROOT}/outputs/cloud_map"

MAP_DIR=""
CSV_PATH=""
MANIFEST_PATH=""
TOPIC="/elevation_map/z_max_points"
RVIZ_FIXED_FRAME=""
VIEWER_PID=""
RVIZ_PID=""
RVIZ_TEMP_CONFIG=""

usage() {
  cat <<'EOF'
Usage: bash scripts/view_elevation_map.sh [MAP_DIR]

Open RViz and show MAP_DIR/global_elevation_map.csv as z_max PointCloud2.

Arguments:
  MAP_DIR   Directory containing global_elevation_map.csv and elevation_manifest.yaml.
            If omitted, the latest outputs/cloud_map/map_* result is used.

Example:
  bash scripts/view_elevation_map.sh
  bash scripts/view_elevation_map.sh outputs/cloud_map/map_YYYYmmdd_HHMMSS
EOF
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

absolute_path() {
  local path="$1"

  if [[ "${path}" == /* ]]; then
    printf '%s' "${path}"
  else
    printf '%s/%s' "${WORKSPACE_ROOT}" "${path}"
  fi
}

source_overlay_if_present() {
  local setup_file="$1"

  if [[ -f "${setup_file}" ]]; then
    # 開発・実機・simulationのoverlay有無に依存せず同じscriptを使えるようにする。
    source "${setup_file}"
  fi
}

source_runtime_environment() {
  local had_nounset=0

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Install ROS 2 ${ROS_DISTRO} first." >&2
    exit 1
  fi

  # ROS setupは未定義変数を参照することがあるため、set -uを一時的に緩める。
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

parse_args() {
  if [[ $# -eq 1 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
    usage
    exit 0
  fi
  if [[ $# -gt 1 ]]; then
    usage >&2
    exit 2
  fi
  if [[ $# -eq 1 ]]; then
    MAP_DIR="$(absolute_path "$1")"
  fi
}

latest_map_dir() {
  local latest=""
  local candidate=""

  # 保存成果物として必要なCSVとmanifestが揃っている最新map directoryだけを自動選択する。
  for candidate in "${CLOUD_MAP_ROOT}"/map_*; do
    if [[ ! -d "${candidate}" ]]; then
      continue
    fi
    if [[ ! -f "${candidate}/global_elevation_map.csv" || ! -f "${candidate}/elevation_manifest.yaml" ]]; then
      continue
    fi
    if [[ -z "${latest}" || "${candidate}" -nt "${latest}" ]]; then
      latest="${candidate}"
    fi
  done

  if [[ -z "${latest}" ]]; then
    echo "No complete elevation map result found under ${CLOUD_MAP_ROOT}/map_*." >&2
    exit 1
  fi
  printf '%s' "${latest}"
}

resolve_inputs() {
  if [[ -z "${MAP_DIR}" ]]; then
    MAP_DIR="$(latest_map_dir)"
  fi
  if [[ ! -d "${MAP_DIR}" ]]; then
    echo "Missing map directory: ${MAP_DIR}" >&2
    exit 1
  fi

  CSV_PATH="${MAP_DIR}/global_elevation_map.csv"
  MANIFEST_PATH="${MAP_DIR}/elevation_manifest.yaml"
  if [[ ! -f "${CSV_PATH}" ]]; then
    echo "Missing global_elevation_map.csv: ${CSV_PATH}" >&2
    exit 1
  fi
  if [[ ! -f "${MANIFEST_PATH}" ]]; then
    echo "Missing elevation_manifest.yaml: ${MANIFEST_PATH}" >&2
    exit 1
  fi
}

manifest_frame_id() {
  local line=""
  local key=""
  local value=""

  # manifestのtop-level frame_idだけを読み、RViz fixed frameとviewer nodeのframe解決を揃える。
  while IFS= read -r line; do
    line="${line%%#*}"
    line="$(trim "${line}")"
    if [[ -z "${line}" || "${line}" != *:* ]]; then
      continue
    fi
    key="$(trim "${line%%:*}")"
    value="$(trim "${line#*:}")"
    if [[ "${key}" == "frame_id" ]]; then
      value="${value%\"}"
      value="${value#\"}"
      value="${value%\'}"
      value="${value#\'}"
      printf '%s' "${value}"
      return 0
    fi
  done < "${MANIFEST_PATH}"
  return 1
}

resolve_rviz_fixed_frame() {
  RVIZ_FIXED_FRAME="$(manifest_frame_id || true)"
  if [[ -z "${RVIZ_FIXED_FRAME}" ]]; then
    RVIZ_FIXED_FRAME="lidar_init"
  fi
}

create_generated_rviz_config() {
  RVIZ_TEMP_CONFIG="$(mktemp "${TMPDIR:-/tmp}/elevation_map_viewer.XXXXXX.rviz")"
  cat > "${RVIZ_TEMP_CONFIG}" <<EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Elevation z_max Points1
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Elevation z_max Points
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.03
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: ${TOPIC}
      Queue Size: 100
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: ${RVIZ_FIXED_FRAME}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.8
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 900
  QMainWindow State: 000000ff00000000fd0000000100000000000001560000035cfc0200000001fb000000100044006900730070006c006100790073010000003d0000035c000000c900ffffff000003200000035c00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Views:
    collapsed: false
  Width: 1280
  X: 80
  Y: 80
EOF
  printf '%s' "${RVIZ_TEMP_CONFIG}"
}

cleanup() {
  local status=$?

  if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${VIEWER_PID}" ]] && kill -0 "${VIEWER_PID}" 2>/dev/null; then
    kill "${VIEWER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_TEMP_CONFIG}" && -f "${RVIZ_TEMP_CONFIG}" ]]; then
    rm -f "${RVIZ_TEMP_CONFIG}"
  fi
  exit "${status}"
}

run_viewer_node() {
  # viewer nodeはmap directoryだけを入力にし、CSVとmanifestは固定ファイル名で読み込む。
  ros2 run ai_ship_robot_slam elevation_map_marker_viewer_node --ros-args \
    -p "map_dir:=${MAP_DIR}" &
  VIEWER_PID=$!
}

ensure_rviz_available() {
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "rviz2 command was not found. Install rviz2." >&2
    exit 1
  fi
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "No graphical display is available. Enable X11/Wayland forwarding before running this viewer." >&2
    exit 1
  fi
}

run_rviz() {
  local rviz_config_path=""

  rviz_config_path="$(create_generated_rviz_config)"
  echo "Opening RViz with fixed_frame=${RVIZ_FIXED_FRAME} config=${rviz_config_path}" >&2
  rviz2 -d "${rviz_config_path}" &
  RVIZ_PID=$!
  wait "${RVIZ_PID}"
}

main() {
  parse_args "$@"
  resolve_inputs
  source_runtime_environment
  resolve_rviz_fixed_frame

  echo "Publishing z_max points from ${CSV_PATH}" >&2
  echo "Using manifest ${MANIFEST_PATH}" >&2
  echo "PointCloud2 topic: ${TOPIC}" >&2
  ensure_rviz_available
  trap cleanup EXIT INT TERM
  run_viewer_node
  run_rviz
}

main "$@"
