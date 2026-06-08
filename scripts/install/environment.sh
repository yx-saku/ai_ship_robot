#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SYSTEM_DEPENDENCIES_SCRIPT="${SCRIPT_DIR}/system_dependencies.sh"
SHELL_ENVIRONMENT_SCRIPT="${SCRIPT_DIR}/shell_environment.sh"
ROS_WS="${WORKSPACE_ROOT}/ros2_ws"
THIRD_PARTY_ROOT="${WORKSPACE_ROOT}/third_party"
THIRD_PARTY_WS="${THIRD_PARTY_ROOT}/ws"
THIRD_PARTY_SRC_DIR="${THIRD_PARTY_WS}/src"
LIVOX_DRIVER_DIR="${THIRD_PARTY_SRC_DIR}/livox_ros_driver2"
LIVOX_SIM_DIR="${THIRD_PARTY_SRC_DIR}/ros2_livox_simulation"
LIVOX_SDK_DIR="${THIRD_PARTY_ROOT}/vendor/Livox-SDK2"
APT_PRIMARY_MIRROR="${APT_PRIMARY_MIRROR:-}"
APT_PORTS_MIRROR="${APT_PORTS_MIRROR:-}"
APT_SECURITY_MIRROR="${APT_SECURITY_MIRROR:-}"
APT_FORCE_IPV4="${APT_FORCE_IPV4:-true}"
APT_RETRIES="${APT_RETRIES:-3}"
APT_HTTP_TIMEOUT="${APT_HTTP_TIMEOUT:-20}"
APT_HTTPS_TIMEOUT="${APT_HTTPS_TIMEOUT:-${APT_HTTP_TIMEOUT}}"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
INSTALL_MODE="full"
APT_UPDATED=0
ROSDEP_UPDATED=0
export DEBIAN_FRONTEND

usage() {
  cat <<'EOF'
Usage: bash scripts/install/environment.sh [OPTIONS]

Options:
  --full           ROS 2 Humble導入、追加依存導入、外部repo取得、workspace buildまで実行する。既定値。
  --system-only    ROS 2 Humble導入、追加apt依存導入、rosdep初期化まで行う。
  --workspace-only 外部repo取得、互換symlink、SDK導入、workspace buildを行う。ROS 2導入済み環境向け。
  --shell-only     aptやbuildは行わず、shell自動読み込み設定だけを更新する。
  -h, --help       このhelpを表示する。

 --workspace-only 以外は、ROS 2 Humble未導入環境でも実行できる。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --full)
      INSTALL_MODE="full"
      ;;
    --system-only)
      INSTALL_MODE="system-only"
      ;;
    --workspace-only)
      INSTALL_MODE="workspace-only"
      ;;
    --shell-only)
      INSTALL_MODE="shell-only"
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Run with --help to see available options." >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required when running as a non-root user." >&2
    exit 1
  fi
  SUDO="sudo"
fi

run_system_dependencies_script() {
  # Dockerfileの重いlayerと通常の手動実行で同じsystem依存導入処理を使う。
  bash "${SYSTEM_DEPENDENCIES_SCRIPT}"
}

run_shell_environment_script() {
  # shell設定だけを独立scriptに分け、system依存導入layerのcacheを壊しにくくする。
  bash "${SHELL_ENVIRONMENT_SCRIPT}"
}

require_ros2() {
  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Run this script without --workspace-only first." >&2
    exit 1
  fi
}

source_ros2() {
  # ROS setup scriptはnounset-safeではないため、読み込み中だけnounsetを解除する。
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
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

  # source処理本体を独立ファイルへ置き、bashrcやprofile側は薄い読み込み口だけにする。
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

  # 再実行時に設定が重複しないよう、前回の管理ブロックだけを取り除いてから先頭へ最新内容を書き込む。
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

  # Ubuntu既定のinteractive判定より前に読み込み口を置き、login shell経由でもROS環境を参照できるようにする。
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

collect_missing_packages() {
  local -n missing_packages_ref="$1"
  shift

  local package_name=""
  local package_status=""
  local normalized_name=""
  declare -A installed_packages=()

  missing_packages_ref=()

  # dpkg-queryをpackageごとに起動せず、まとめて状態を取得して再実行時の小さな待ち時間を減らす。
  while IFS=$'\t' read -r package_name package_status; do
    if [[ "${package_status}" == "install ok installed" ]]; then
      normalized_name="${package_name%%:*}"
      installed_packages["${package_name}"]=1
      installed_packages["${normalized_name}"]=1
    fi
  done < <(dpkg-query -W -f='${Package}\t${Status}\n' "$@" 2>/dev/null || true)

  # aptへ渡す入力順を保ち、不足packageだけをinstall対象にする。
  for package_name in "$@"; do
    if [[ "${installed_packages[${package_name}]:-0}" -ne 1 ]]; then
      missing_packages_ref+=("${package_name}")
    fi
  done
}

locale_generated() {
  local locale_name="$1"

  locale -a 2>/dev/null | grep -Eqi "^${locale_name}$"
}

ensure_locales_generated() {
  local needs_locale_gen=0

  if ! command -v locale-gen >/dev/null 2>&1; then
    return 0
  fi

  # 生成済みlocaleを再生成しないことで、再実行時のlocale-gen待ちを避ける。
  if ! locale_generated "en_US\.utf8" || ! locale_generated "ja_JP\.utf8"; then
    needs_locale_gen=1
  fi

  if [[ "${needs_locale_gen}" -eq 1 ]]; then
    ${SUDO} locale-gen en_US en_US.UTF-8 ja_JP.UTF-8
  fi

  if [[ ! -f /etc/default/locale ]] || ! grep -Eq '^LANG=en_US\.UTF-8$|^LANG="en_US\.UTF-8"$' /etc/default/locale; then
    ${SUDO} update-locale LANG=en_US.UTF-8
  fi
}

mark_apt_indexes_stale() {
  APT_UPDATED=0
}

apt_update_if_needed() {
  if [[ "${APT_UPDATED}" -eq 1 ]]; then
    return 0
  fi

  run_with_safe_apt_locale apt-get update
  APT_UPDATED=1
}

run_with_safe_apt_locale() {
  # locales導入前でもdpkg maintainer scriptが未生成localeを参照しないよう、安全なUTF-8 localeでapt系処理を実行する。
  ${SUDO} env \
    DEBIAN_FRONTEND="${DEBIAN_FRONTEND}" \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    "$@"
}

install_apt_packages_if_missing() {
  local missing_packages=()

  # 再実行時の待ち時間を減らすため、未導入packageがある時だけapt installを実行する。
  collect_missing_packages missing_packages "$@"

  if [[ "${#missing_packages[@]}" -eq 0 ]]; then
    return 0
  fi

  apt_update_if_needed
  run_with_safe_apt_locale apt-get install -y --no-install-recommends "${missing_packages[@]}"
}

write_apt_network_config() {
  local apt_config_path="/etc/apt/apt.conf.d/99ai-ship-robot-network"
  local temp_config=""

  temp_config="$(mktemp)"

  # 遅いIPv6経路や不安定な回線でaptが長時間停止しないよう、workspace専用の通信設定を固定する。
  {
    printf 'Acquire::Retries "%s";\n' "${APT_RETRIES}"
    printf 'Acquire::http::Timeout "%s";\n' "${APT_HTTP_TIMEOUT}"
    printf 'Acquire::https::Timeout "%s";\n' "${APT_HTTPS_TIMEOUT}"

    if [[ "${APT_FORCE_IPV4}" == "true" ]]; then
      printf 'Acquire::ForceIPv4 "true";\n'
    fi
  } > "${temp_config}"

  ${SUDO} install -m 0644 "${temp_config}" "${apt_config_path}"
  rm -f "${temp_config}"
}

backup_file_once() {
  local file_path="$1"
  local backup_path="${file_path}.ai_ship_robot.bak"

  if [[ -f "${backup_path}" || ! -f "${file_path}" ]]; then
    return 0
  fi

  ${SUDO} cp "${file_path}" "${backup_path}"
}

replace_apt_source_url() {
  local current_url="$1"
  local replacement_url="$2"
  local temp_sources=""

  if [[ -z "${replacement_url}" || ! -f "/etc/apt/sources.list" ]]; then
    return 0
  fi

  if ! grep -Fq "${current_url}" /etc/apt/sources.list; then
    return 0
  fi

  temp_sources="$(mktemp)"
  sed "s#${current_url}#${replacement_url%/}#g" /etc/apt/sources.list > "${temp_sources}"

  if cmp -s "${temp_sources}" /etc/apt/sources.list; then
    rm -f "${temp_sources}"
    return 0
  fi

  backup_file_once "/etc/apt/sources.list"
  ${SUDO} install -m 0644 "${temp_sources}" /etc/apt/sources.list
  rm -f "${temp_sources}"
  mark_apt_indexes_stale
}

configure_ubuntu_apt_mirrors() {
  # 社内回線と相性の悪いmirrorを避けられるよう、必要時だけsources.listのURLを差し替える。
  replace_apt_source_url "http://archive.ubuntu.com/ubuntu" "${APT_PRIMARY_MIRROR}"
  replace_apt_source_url "https://archive.ubuntu.com/ubuntu" "${APT_PRIMARY_MIRROR}"
  replace_apt_source_url "http://ports.ubuntu.com/ubuntu-ports" "${APT_PORTS_MIRROR}"
  replace_apt_source_url "https://ports.ubuntu.com/ubuntu-ports" "${APT_PORTS_MIRROR}"
  replace_apt_source_url "http://security.ubuntu.com/ubuntu" "${APT_SECURITY_MIRROR}"
  replace_apt_source_url "https://security.ubuntu.com/ubuntu" "${APT_SECURITY_MIRROR}"
}

apt_source_has_component() {
  local component_name="$1"

  grep -Eqs "^[[:space:]]*deb .*[[:space:]]${component_name}([[:space:]]|$)" /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null \
    || grep -Eqs "^[[:space:]]*Components: .*[[:space:]]?${component_name}([[:space:]]|$)" /etc/apt/sources.list.d/*.sources 2>/dev/null
}

enable_apt_component_in_sources_list() {
  local component_name="$1"
  local sources_file="/etc/apt/sources.list"
  local temp_sources=""
  local line=""
  local changed=0

  if apt_source_has_component "${component_name}"; then
    return 0
  fi

  if [[ ! -f "${sources_file}" ]]; then
    echo "Missing ${sources_file}; cannot enable apt component: ${component_name}" >&2
    return 1
  fi

  temp_sources="$(mktemp)"

  # software-properties-commonを入れず、Ubuntuのdeb行へ必要なcomponentだけを直接追加して依存導入を軽くする。
  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ "${line}" =~ ^[[:space:]]*deb[[:space:]] ]] \
      && [[ "${line}" =~ [[:space:]]main([[:space:]]|$) ]] \
      && [[ ! "${line}" =~ [[:space:]]${component_name}([[:space:]]|$) ]]; then
      printf '%s %s\n' "${line}" "${component_name}" >> "${temp_sources}"
      changed=1
      continue
    fi

    printf '%s\n' "${line}" >> "${temp_sources}"
  done < "${sources_file}"

  if [[ "${changed}" -eq 0 ]]; then
    rm -f "${temp_sources}"
    echo "No editable apt source line found for component: ${component_name}" >&2
    return 1
  fi

  backup_file_once "${sources_file}"
  ${SUDO} install -m 0644 "${temp_sources}" "${sources_file}"
  rm -f "${temp_sources}"
  mark_apt_indexes_stale
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

  if [[ ! "${ROSDEP_UPDATE_MAX_AGE_SECONDS}" =~ ^[0-9]+$ || "${ROSDEP_UPDATE_MAX_AGE_SECONDS}" -eq 0 ]]; then
    return 1
  fi

  if [[ ! -f "${stamp_file}" || ! -d "${cache_dir}" ]]; then
    return 1
  fi

  updated_at="$(<"${stamp_file}")"
  if [[ ! "${updated_at}" =~ ^[0-9]+$ ]]; then
    return 1
  fi

  # rosdep cacheをbind mountで保持し、一定時間内の再作成コンテナではネットワーク更新を省略する。
  now="$(date +%s)"
  (( now - updated_at < ROSDEP_UPDATE_MAX_AGE_SECONDS ))
}

ensure_rosdep_updated() {
  if ! command -v rosdep >/dev/null 2>&1; then
    return 0
  fi

  ensure_rosdep_initialized

  if [[ "${ROSDEP_UPDATED}" -eq 1 ]]; then
    return 0
  fi

  if rosdep_cache_fresh; then
    echo "Skip rosdep update because cache is fresh. Set ROSDEP_UPDATE_MAX_AGE_SECONDS=0 to force update."
    ROSDEP_UPDATED=1
    return 0
  fi

  # rosdep indexはworkspace処理前にだけ更新し、system依存導入時の不要なネットワーク待ちを避ける。
  rosdep update
  mkdir -p "${HOME}/.ros/rosdep"
  date +%s > "${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  ROSDEP_UPDATED=1
}

install_system_dependencies() {
  local ros_packages=(
    "ros-${ROS_DISTRO}-ros-base"
    "ros-${ROS_DISTRO}-ament-cmake"
    "ros-${ROS_DISTRO}-ament-cmake-auto"
    "ros-${ROS_DISTRO}-diagnostic-msgs"
    "ros-${ROS_DISTRO}-gazebo-dev"
    "ros-${ROS_DISTRO}-gazebo-plugins"
    "ros-${ROS_DISTRO}-gazebo-ros"
    "ros-${ROS_DISTRO}-launch"
    "ros-${ROS_DISTRO}-launch-ros"
    "ros-${ROS_DISTRO}-message-filters"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-robot-state-publisher"
    "ros-${ROS_DISTRO}-ros2launch"
    "ros-${ROS_DISTRO}-ros2run"
    "ros-${ROS_DISTRO}-ros2service"
    "ros-${ROS_DISTRO}-ros2topic"
    "ros-${ROS_DISTRO}-rosbag2"
    "ros-${ROS_DISTRO}-rosidl-default-generators"
    "ros-${ROS_DISTRO}-rviz2"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-tf2-sensor-msgs"
    "ros-${ROS_DISTRO}-xacro"
  )
  local system_packages=(
    build-essential
    cmake
    gazebo
    git
    libapr1-dev
    libaprutil1-dev
    libboost-chrono-dev
    libboost-dev
    libeigen3-dev
    libgazebo-dev
    libpcl-dev
    libprotobuf-dev
    protobuf-compiler
    python3-colcon-common-extensions
    python3-rosdep
    python3-vcstool
    sudo
  )

  # 本番Jetsonと開発Dockerで同じインストールスクリプトを使うため、ROS 2本体と追加依存を同じ場所で導入する。
  install_apt_packages_if_missing "${system_packages[@]}" "${ros_packages[@]}"

  ensure_locales_generated

  if command -v rosdep >/dev/null 2>&1; then
    # system-onlyではcache更新まで行わず、workspaceのrosdep install直前に必要な場合だけ更新する。
    ensure_rosdep_initialized
  fi
}

configure_ros2_apt_repository() {
  local prerequisite_packages=(
    ca-certificates
    curl
    gnupg
    locales
    tzdata
  )
  local ros_keyring_path="/etc/apt/keyrings/ros-archive-keyring.gpg"
  local ros_repo_file="/etc/apt/sources.list.d/ros2.list"
  local ros_repo_line=""

  ros_repo_line="deb [arch=$(dpkg --print-architecture) signed-by=${ros_keyring_path}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main"

  write_apt_network_config
  configure_ubuntu_apt_mirrors

  # Dockerと実機で同じ入口に寄せるため、ROS 2 apt repository 設定も install script 側へ集約する。
  install_apt_packages_if_missing "${prerequisite_packages[@]}"

  # 既存sourcesですでに有効なcomponentは触らず、不要なrepository更新を避ける。
  if ! apt_source_has_component universe; then
    enable_apt_component_in_sources_list universe
  fi

  ${SUDO} install -m 0755 -d /etc/apt/keyrings

  if [[ ! -f "${ros_keyring_path}" ]]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | ${SUDO} tee "${ros_keyring_path}" >/dev/null
  fi

  if [[ ! -f "${ros_repo_file}" ]] || [[ "$(<"${ros_repo_file}")" != "${ros_repo_line}" ]]; then
    printf '%s\n' "${ros_repo_line}" | ${SUDO} tee "${ros_repo_file}" >/dev/null
    mark_apt_indexes_stale
  fi
}

prepare_livox_simulation_build_environment() {
  local multiarch_dir="/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)"

  # upstream の固定ライブラリ名依存を減らすため、repoを広く書き換えずに Jammy/Humble 側で互換symlinkを用意する。
  if [[ -f "${multiarch_dir}/libprotobuf.so" && ! -e "${multiarch_dir}/libprotobuf.so.9" ]]; then
    ${SUDO} ln -s libprotobuf.so "${multiarch_dir}/libprotobuf.so.9"
  fi

  # upstream の固定ライブラリ名依存を減らすため、repoを広く書き換えずに Jammy/Humble 側で互換symlinkを用意する。
  if [[ -f "${multiarch_dir}/libboost_chrono.so" && ! -e "${multiarch_dir}/libboost_chrono.so.1.71.0" ]]; then
    ${SUDO} ln -s libboost_chrono.so "${multiarch_dir}/libboost_chrono.so.1.71.0"
  fi

  # upstream の固定 include path を崩さず使えるように、環境側で存在確認だけを行う。
  if [[ ! -d "/usr/include/gazebo-11/gazebo" ]]; then
    echo "Missing /usr/include/gazebo-11/gazebo. gazebo development headers are required." >&2
    return 1
  fi

  ${SUDO} ldconfig
}

ensure_git_repo() {
  local target_dir="$1"
  local repo_url="$2"

  if [[ -d "${target_dir}/.git" ]]; then
    return 0
  fi

  mkdir -p "$(dirname "${target_dir}")"
  git clone --depth 1 "${repo_url}" "${target_dir}"
}

patch_ros2_livox_simulation_repo() {
  local target_dir="$1"
  local marker_file="$2"

  # 既存マーカーがある環境でも追加修正を取り込めるよう、冪等な文字列置換を毎回実行する。
  TARGET_DIR="${target_dir}" python3 <<'PY'
from pathlib import Path
import os

target_dir = Path(os.environ["TARGET_DIR"])

plugin_path = target_dir / "src" / "livox_points_plugin.cpp"
original_plugin_text = plugin_path.read_text(encoding="utf-8")
plugin_text = original_plugin_text
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
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"custom topic name: %s\", custom_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"pointcloud2 topic name: %s\", pointcloud2_topic.c_str());\n"
    "        RCLCPP_INFO(rclcpp::get_logger(\"LivoxPointsPlugin\"), \"ros topic name: %s\", curr_scan_topic.c_str());",
)
plugin_text = plugin_text.replace(
    "        // PointCloud2 publisher\n"
    "        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + \"_PointCloud2\", 10);\n"
    "        // CustomMsg publisher\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);",
    "        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud2_topic, 10);\n"
    "        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(custom_topic, 10);",
)

# 点群の重複追加は環境側では吸収できないため、plugin 実装に限定して最小差分で修正する。
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

# ray開始点がminDist分前方にあるため、publishする点群はLiDAR原点基準へ補正する。
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

prepare_livox_ros_driver2_manifest() {
  local ros2_manifest="${LIVOX_DRIVER_DIR}/package_ROS2.xml"
  local active_manifest="${LIVOX_DRIVER_DIR}/package.xml"

  if [[ ! -f "${ros2_manifest}" ]]; then
    echo "Missing ROS2 manifest: ${ros2_manifest}" >&2
    return 1
  fi

  # upstream build.sh相当のmanifest切替は、内容が変わる場合だけ行いmtime起因の再configureを避ける。
  if [[ ! -f "${active_manifest}" ]] || ! cmp -s "${ros2_manifest}" "${active_manifest}"; then
    cp "${ros2_manifest}" "${active_manifest}"
  fi
}

install_livox_sdk2_if_needed() {
  local sdk_build_dir="${LIVOX_SDK_DIR}/build"
  local livox_sdk_cxx_flags="${CMAKE_CXX_FLAGS:-}"

  if [[ -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]]; then
    return 0
  fi

  if [[ ! -d "${LIVOX_SDK_DIR}" ]]; then
    echo "Missing Livox-SDK2 repository at ${LIVOX_SDK_DIR}" >&2
    return 1
  fi

  # Livox-SDK2同梱RapidJSONの古いpragma抑制記述がGCCで大量の-Wpragmasログを出すため、SDKソースを変更せずビルド時だけ抑制する。
  if [[ " ${livox_sdk_cxx_flags} " != *" -Wno-pragmas "* ]]; then
    livox_sdk_cxx_flags="${livox_sdk_cxx_flags:+${livox_sdk_cxx_flags} }-Wno-pragmas"
  fi

  # 公式driverが要求する共有SDKを先に入れ、upstream build.shのlink失敗を防ぐ。
  cmake -S "${LIVOX_SDK_DIR}" -B "${sdk_build_dir}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="${livox_sdk_cxx_flags}"
  cmake --build "${sdk_build_dir}" --parallel
  ${SUDO} cmake --install "${sdk_build_dir}"
  ${SUDO} ldconfig
}

install_rosdeps_for_workspace() {
  local workspace_src="$1"

  if ! command -v rosdep >/dev/null 2>&1; then
    return 0
  fi

  ensure_rosdep_updated
  rosdep install --from-paths "${workspace_src}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

clone_external_repositories() {
  # 外部repoはgit管理外の専用領域に置き、upstreamの構成をできるだけ維持する。
  ensure_git_repo "${LIVOX_DRIVER_DIR}" "https://github.com/Livox-SDK/livox_ros_driver2.git"
  ensure_git_repo "${LIVOX_SIM_DIR}" "https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git"
  ensure_git_repo "${LIVOX_SDK_DIR}" "https://github.com/Livox-SDK/Livox-SDK2.git"
  patch_ros2_livox_simulation_repo "${LIVOX_SIM_DIR}" "${LIVOX_SIM_DIR}/.ai_ship_robot_patch_applied"
}

setup_file_has_stale_third_party_path() {
  local setup_file="$1"

  [[ -f "${setup_file}" ]] || return 1
  grep -Fq "${WORKSPACE_ROOT}/third_party_ws" "${setup_file}" \
    || grep -Fq "${WORKSPACE_ROOT}/third_party_vendor" "${setup_file}"
}

remove_stale_workspace_artifacts() {
  local stale_artifacts=0

  if setup_file_has_stale_third_party_path "${THIRD_PARTY_WS}/install/setup.bash" \
    || setup_file_has_stale_third_party_path "${ROS_WS}/install/setup.bash"; then
    stale_artifacts=1
  fi

  if [[ "${stale_artifacts}" -eq 0 ]]; then
    return 0
  fi

  # third_party配下へ移動する前の絶対パスを含むcolcon生成物は、source時に旧パスを参照するため再生成する。
  echo "Remove stale workspace artifacts that reference old third_party paths."
  rm -rf \
    "${THIRD_PARTY_WS}/build" \
    "${THIRD_PARTY_WS}/install" \
    "${THIRD_PARTY_WS}/log" \
    "${ROS_WS}/build" \
    "${ROS_WS}/install" \
    "${ROS_WS}/log"
}

build_third_party_workspace() {
  if [[ ! -d "${LIVOX_DRIVER_DIR}" ]]; then
    echo "Missing livox_ros_driver2 repository at ${LIVOX_DRIVER_DIR}" >&2
    return 1
  fi
  if [[ ! -d "${LIVOX_SIM_DIR}" ]]; then
    echo "Missing ros2_livox_simulation repository at ${LIVOX_SIM_DIR}" >&2
    return 1
  fi

  # upstream build.shは毎回build/installを削除するため、直接colconを実行してキャッシュを保持する。
  colcon \
    --log-base "${THIRD_PARTY_WS}/log" \
    build \
    --base-paths "${THIRD_PARTY_SRC_DIR}" \
    --build-base "${THIRD_PARTY_WS}/build" \
    --install-base "${THIRD_PARTY_WS}/install" \
    --cmake-args \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS="${ROS_DISTRO}"
}

build_project_workspace() {
  # 通常のcolcon buildにして、Python/URDFのみの変更でも既存キャッシュを保持する。
  colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS}/src" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
}

install_workspace() {
  source_ros2
  clone_external_repositories
  remove_stale_workspace_artifacts
  prepare_livox_simulation_build_environment
  install_livox_sdk2_if_needed
  prepare_livox_ros_driver2_manifest
  install_rosdeps_for_workspace "${THIRD_PARTY_SRC_DIR}"
  build_third_party_workspace

  # third_party overlayを重ねてからプロジェクトworkspaceのrosdep解決とビルドを行う。
  set +u
  source "${THIRD_PARTY_WS}/install/setup.bash"
  set -u
  install_rosdeps_for_workspace "${ROS_WS}/src"
  build_project_workspace
}

case "${INSTALL_MODE}" in
  full)
    run_system_dependencies_script
    require_ros2
    install_workspace
    run_shell_environment_script
    ;;
  system-only)
    run_system_dependencies_script
    run_shell_environment_script
    ;;
  workspace-only)
    require_ros2
    install_workspace
    run_shell_environment_script
    ;;
  shell-only)
    run_shell_environment_script
    ;;
esac
