#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
THIRD_PARTY_WS="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws"
THIRD_PARTY_SRC_DIR="${THIRD_PARTY_WS}/src"
THIRD_PARTY_BUILD_DIR="${THIRD_PARTY_WS}/build"
THIRD_PARTY_INSTALL_DIR="${THIRD_PARTY_WS}/install"
THIRD_PARTY_LOG_DIR="${THIRD_PARTY_WS}/log"
LIVOX_SIM_DIR="${THIRD_PARTY_SRC_DIR}/ros2_livox_simulation"
LIVOX_SIM_REF="58ae16b43cc90423d3f8dc2ae3018a7c178c330a"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
ROSDEP_UPDATED=0

usage() {
  cat <<'EOF'
Usage: bash sim/install/install_third_party.sh

simulation専用の third_party を system 側の ROS underlay へ追加導入します。
- 事前に bash install/install_third_party.sh を実行してください。
- ros2_livox_simulation を /opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws へ追加します。
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

source_ros2() {
  # ROS setup scriptはnounset-safeではないため、読み込み中だけnounsetを解除する。
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

ensure_owned_directory() {
  local directory_path="$1"

  # 共通underlayと同じworkspaceへsimulation packageを追加するため、必要な作業領域だけ用意する。
  if [[ "$(id -u)" -eq 0 ]]; then
    mkdir -p "${directory_path}"
  else
    ${SUDO} install -d -o "$(id -u)" -g "$(id -g)" "${directory_path}"
    ${SUDO} chown -R "$(id -u):$(id -g)" "${directory_path}"
  fi
}

prepare_install_roots() {
  # colconのsrc/build/install/logを同じunderlay workspace内に揃える。
  ensure_owned_directory "${THIRD_PARTY_SRC_DIR}"
  ensure_owned_directory "${THIRD_PARTY_BUILD_DIR}"
  ensure_owned_directory "${THIRD_PARTY_INSTALL_DIR}"
  ensure_owned_directory "${THIRD_PARTY_LOG_DIR}"
}

directory_is_empty() {
  local directory_path="$1"
  local entries=()

  # 失敗した前回実行で空のclone先だけが残った場合は、再実行でそのまま初期化できるようにする。
  shopt -s nullglob dotglob
  entries=("${directory_path}"/*)
  shopt -u nullglob dotglob
  [[ "${#entries[@]}" -eq 0 ]]
}

ensure_git_repo() {
  local target_dir="$1"
  local repo_url="$2"
  local expected_ref="$3"
  local current_ref=""

  if [[ -d "${target_dir}/.git" ]]; then
    current_ref="$(git -C "${target_dir}" rev-parse HEAD)"
    if [[ "${current_ref}" != "${expected_ref}" ]]; then
      echo "Unexpected repository revision at ${target_dir}" >&2
      echo "  expected: ${expected_ref}" >&2
      echo "  current : ${current_ref}" >&2
      echo "Remove the directory or update the pinned ref after validating the new revision." >&2
      return 1
    fi
    return 0
  fi

  if [[ -e "${target_dir}" ]]; then
    if [[ ! -d "${target_dir}" ]] || ! directory_is_empty "${target_dir}"; then
      echo "Existing path is not a git repository: ${target_dir}" >&2
      return 1
    fi
  fi

  # simulation専用repoも検証済みcommitへ固定し、再現性を維持する。
  mkdir -p "$(dirname "${target_dir}")"
  git init "${target_dir}"
  git -C "${target_dir}" remote add origin "${repo_url}"
  git -C "${target_dir}" fetch --depth 1 origin "${expected_ref}"
  git -C "${target_dir}" checkout --detach FETCH_HEAD
}

patch_ros2_livox_simulation_repo() {
  local target_dir="$1"
  local marker_file="$2"
  local plugin_path="${target_dir}/src/livox_points_plugin.cpp"

  if [[ ! -f "${plugin_path}" ]]; then
    echo "Missing Livox simulation plugin source: ${plugin_path}" >&2
    return 1
  fi

  # 非公式pluginへ必要なtopic分離とLiDAR原点補正だけを最小差分で冪等適用する。
  TARGET_DIR="${target_dir}" python3 <<'PY'
from pathlib import Path
import os

target_dir = Path(os.environ["TARGET_DIR"])
plugin_path = target_dir / "src" / "livox_points_plugin.cpp"
original_plugin_text = plugin_path.read_text(encoding="utf-8")
plugin_text = original_plugin_text

# CustomMsgとPointCloud2のtopic名を別々に指定できるようにする。
plugin_text = plugin_text.replace(
    "        auto curr_scan_topic = sdf->Get<std::string>(\"topic\");\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"ros topic name: %s\", curr_scan_topic.c_str());",
    "        auto curr_scan_topic = sdf->Get<std::string>(\"topic\");\n"
    "        auto custom_topic = curr_scan_topic;\n"
    "        auto pointcloud2_topic = curr_scan_topic + \"_PointCloud2\";\n"
    "        if (sdf->HasElement(\"custom_topic\"))\n"
    "        {\n"
    "            custom_topic = sdf->Get<std::string>(\"custom_topic\");\n"
    "        }\n"
    "        if (sdf->HasElement(\"pointcloud2_topic\"))\n"
    "        {\n"
    "            pointcloud2_topic = sdf->Get<std::string>(\"pointcloud2_topic\");\n"
    "        }\n"
    "        bool enable_pointcloud2 = true;\n"
    "        if (sdf->HasElement(\"enable_pointcloud2\"))\n"
    "        {\n"
    "            enable_pointcloud2 = sdf->Get<bool>(\"enable_pointcloud2\");\n"
    "        }\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"custom topic name: %s\", custom_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"pointcloud2 topic name: %s\", pointcloud2_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"ros topic name: %s\", curr_scan_topic.c_str());",
)
plugin_text = plugin_text.replace(
    "        // PointCloud2 publisher\n"
    "        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + \"_PointCloud2\", 10);\n"
    "        // CustomMsg publisher\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);",
    "        if (enable_pointcloud2)\n"
    "        {\n"
    "            cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud2_topic, 10);\n"
    "        }\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(custom_topic, 10);",
)
plugin_text = plugin_text.replace(
    "        cloud2_pub->publish(cloud2);",
    "        if (cloud2_pub)\n"
    "        {\n"
    "            cloud2_pub->publish(cloud2);\n"
    "        }",
)

# 無効値が原点付近の点として二重追加される処理を取り除く。
plugin_text = plugin_text.replace(
    "            // Fill the PointCloud point cloud message\n"
    "            clouds.emplace_back();\n"
    "            clouds.back().x = point.X();\n"
    "            clouds.back().y = point.Y();\n"
    "            clouds.back().z = point.Z();\n\n"
    "            // Fill the PointCloud point cloud message\n",
    "            // Fill the PointCloud point cloud message\n",
    1,
)

# Gazebo rayのminDist補正を反映し、LiDAR原点基準の点群にそろえる。
plugin_text = plugin_text.replace(
    "            // Handle out-of-range data\n"
    "            if (range >= RangeMax())\n"
    "            {\n"
    "                range = 0;\n"
    "            }\n"
    "            else if (range <= RangeMin())\n"
    "            {\n"
    "                range = 0;\n"
    "            }\n\n"
    "            // Calculate point cloud data\n"
    "            auto rotate_info = pair.second;\n"
    "            ignition::math::Quaterniond ray;\n"
    "            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));\n"
    "            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);\n"
    "            auto point = range * axis;",
    "            // GazeboのrayはminDistだけ前方から始まるため、LiDAR原点基準の距離へ戻す。\n"
    "            const auto corrected_range = minDist + range;\n\n"
    "            // 無効な計測値は原点付近の点として可視化されるため、点群へ追加しない。\n"
    "            if (corrected_range >= RangeMax() || corrected_range <= RangeMin())\n"
    "            {\n"
    "                continue;\n"
    "            }\n\n"
    "            // 補正済み距離を使い、ROSへ出す点群をLiDAR frame基準で生成する。\n"
    "            auto rotate_info = pair.second;\n"
    "            ignition::math::Quaterniond ray;\n"
    "            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));\n"
    "            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);\n"
    "            auto point = corrected_range * axis;",
)

if plugin_text != original_plugin_text:
    plugin_path.write_text(plugin_text, encoding="utf-8")
PY
  touch "${marker_file}"
}

clone_simulation_repositories() {
  # simulation専用のGazebo plugin repoだけを共通underlay workspaceへ追加する。
  ensure_git_repo "${LIVOX_SIM_DIR}" "https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git" "${LIVOX_SIM_REF}"
  patch_ros2_livox_simulation_repo "${LIVOX_SIM_DIR}" "${LIVOX_SIM_DIR}/.ai_ship_robot_patch_applied"
}

prepare_livox_simulation_build_environment() {
  local multiarch_dir="/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"

  # upstream固定ライブラリ名依存を吸収するため、環境側で互換symlinkを用意する。
  if [[ -f "${multiarch_dir}/libprotobuf.so" && ! -e "${multiarch_dir}/libprotobuf.so.9" ]]; then
    ${SUDO} ln -s libprotobuf.so "${multiarch_dir}/libprotobuf.so.9"
  fi
  # upstream固定ライブラリ名依存を吸収するため、環境側で互換symlinkを用意する。
  if [[ -f "${multiarch_dir}/libboost_chrono.so" && ! -e "${multiarch_dir}/libboost_chrono.so.1.71.0" ]]; then
    ${SUDO} ln -s libboost_chrono.so "${multiarch_dir}/libboost_chrono.so.1.71.0"
  fi
  # 固定include path前提のpluginをそのまま使うため、環境側でheader存在を確認する。
  if [[ ! -d "/usr/include/gazebo-11/gazebo" ]]; then
    echo "Missing /usr/include/gazebo-11/gazebo. gazebo development headers are required." >&2
    return 1
  fi
  ${SUDO} ldconfig
}

source_common_underlay() {
  if [[ ! -f "${THIRD_PARTY_INSTALL_DIR}/setup.bash" ]]; then
    echo "Missing common third-party underlay: ${THIRD_PARTY_INSTALL_DIR}/setup.bash" >&2
    echo "Run bash install/install_third_party.sh first." >&2
    return 1
  fi

  # simulation pluginはlivox_ros_driver2のmessageを使うため、共通underlayを先に読み込む。
  set +u
  source "${THIRD_PARTY_INSTALL_DIR}/setup.bash"
  set -u
  if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    echo "Missing livox_ros_driver2 in common third-party underlay." >&2
    echo "Run bash install/install_third_party.sh first." >&2
    return 1
  fi
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

  # simulation third_partyの依存解決直前だけrosdep indexを更新する。
  rosdep update
  mkdir -p "${HOME}/.ros/rosdep"
  date +%s > "${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  ROSDEP_UPDATED=1
}

install_rosdeps_for_paths() {
  local paths=("$@")
  local existing_paths=()
  local path=""

  if ! command -v rosdep >/dev/null 2>&1; then return 0; fi
  for path in "${paths[@]}"; do
    if [[ -d "${path}" ]]; then
      existing_paths+=("${path}")
    fi
  done
  if [[ "${#existing_paths[@]}" -eq 0 ]]; then return 0; fi

  # Gazebo plugin側の不足依存をbuild前に解決し、package単位の失敗を減らす。
  ensure_rosdep_updated
  rosdep install --from-paths "${existing_paths[@]}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

build_third_party_paths() {
  local cxx_standard="$1"
  shift
  local paths=("$@")

  if [[ "${#paths[@]}" -eq 0 ]]; then return 0; fi
  source_common_underlay

  # simulation repoだけを追加buildし、共通underlayの成果物は再buildしない。
  colcon --log-base "${THIRD_PARTY_LOG_DIR}" build \
    --base-paths "${paths[@]}" \
    --build-base "${THIRD_PARTY_BUILD_DIR}" \
    --install-base "${THIRD_PARTY_INSTALL_DIR}" \
    --cmake-args \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_WITH_TBB=OFF \
      -DBUILD_WITH_CUDA=OFF \
      -DBUILD_WITH_VIEWER=OFF \
      -DBUILD_WITH_OPENCV=OFF \
      -DCMAKE_CXX_STANDARD="${cxx_standard}" \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS="${ROS_DISTRO}"
}

build_simulation_underlay_workspace() {
  if [[ ! -d "${LIVOX_SIM_DIR}" ]]; then
    echo "Missing Livox simulation repository under ${THIRD_PARTY_SRC_DIR}" >&2
    return 1
  fi

  # Livox simulation pluginを共通driver/messagesが入ったunderlayへ追加する。
  build_third_party_paths 17 "${LIVOX_SIM_DIR}"
}

install_simulation_third_party() {
  source_ros2
  prepare_install_roots
  source_common_underlay
  prepare_livox_simulation_build_environment
  clone_simulation_repositories
  install_rosdeps_for_paths "${LIVOX_SIM_DIR}"
  build_simulation_underlay_workspace
  echo "Installed simulation third-party underlay: ${THIRD_PARTY_INSTALL_DIR}"
}

require_ros2
install_simulation_third_party
