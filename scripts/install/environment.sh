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
GTSAM_DIR="${THIRD_PARTY_SRC_DIR}/gtsam"
GLIM_DIR="${THIRD_PARTY_SRC_DIR}/glim"
GLIM_ROS2_DIR="${THIRD_PARTY_SRC_DIR}/glim_ros2"
GTSAM_POINTS_DIR="${THIRD_PARTY_SRC_DIR}/gtsam_points"
LIVOX_DRIVER_REF="13eb05e4e6dd7a765b934d0c5fd6236676a57b49"
LIVOX_SIM_REF="58ae16b43cc90423d3f8dc2ae3018a7c178c330a"
LIVOX_SDK_REF="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
GTSAM_REF="4f66a491ffc83cf092d0d818b11dc35135521612"
GTSAM_POINTS_REF="42e811a50e421390c206b486572b88a024734146"
GLIM_REF="25ad190776f05f6a8d7438686197294f73c5f868"
GLIM_ROS2_REF="a62811dc3ab73076f4a43fc21005f96cd712903c"
APT_PRIMARY_MIRROR="${APT_PRIMARY_MIRROR:-}"
APT_PORTS_MIRROR="${APT_PORTS_MIRROR:-}"
APT_SECURITY_MIRROR="${APT_SECURITY_MIRROR:-}"
APT_FORCE_IPV4="${APT_FORCE_IPV4:-true}"
APT_RETRIES="${APT_RETRIES:-3}"
APT_HTTP_TIMEOUT="${APT_HTTP_TIMEOUT:-20}"
APT_HTTPS_TIMEOUT="${APT_HTTPS_TIMEOUT:-${APT_HTTP_TIMEOUT}}"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
INSTALL_MODE="full"
INSTALL_PROFILE="dev"
INSTALL_GROUPS=""
APT_UPDATED=0
ROSDEP_UPDATED=0
export DEBIAN_FRONTEND

declare -A SELECTED_GROUPS=()

usage() {
  cat <<'EOF'
Usage: bash scripts/install/environment.sh [OPTIONS]

Options:
  --full           ROS 2 Humble導入、追加依存導入、外部repo取得、workspace buildまで実行する。既定値。
  --system-only    ROS 2 Humble導入、追加apt依存導入、rosdep初期化まで行う。
  --workspace-only 外部repo取得、互換symlink、SDK導入、workspace buildを行う。ROS 2導入済み環境向け。
  --shell-only     aptやbuildは行わず、shell自動読み込み設定だけを更新する。
  --profile NAME   Install profile: real, simulation, slam-sim, dev. 既定値はdev。
  --groups LIST    Comma-separated groups: base, simulation, glim, dev-test. --profileより優先する。
  -h, --help       このhelpを表示する。

  --workspace-only 以外は、ROS 2 Humble未導入環境でも実行できる。
EOF
}

profile_to_groups() {
  local profile_name="$1"

  case "${profile_name}" in
    real)
      printf '%s' "base,glim"
      ;;
    simulation)
      printf '%s' "base,simulation"
      ;;
    slam-sim)
      printf '%s' "base,simulation,glim"
      ;;
    dev)
      printf '%s' "base,simulation,glim,dev-test"
      ;;
    *)
      echo "Unknown install profile: ${profile_name}" >&2
      echo "Available profiles: real, simulation, slam-sim, dev" >&2
      exit 2
      ;;
  esac
}

select_install_groups() {
  local raw_groups="$1"
  local group_name=""
  local selected_list=()

  SELECTED_GROUPS=()
  IFS=',' read -ra selected_list <<< "${raw_groups}"
  for group_name in "${selected_list[@]}"; do
    group_name="${group_name//[[:space:]]/}"
    [[ -n "${group_name}" ]] || continue
    case "${group_name}" in
      base|simulation|glim|dev-test)
        SELECTED_GROUPS["${group_name}"]=1
        ;;
      *)
        echo "Unknown install group: ${group_name}" >&2
        echo "Available groups: base, simulation, glim, dev-test" >&2
        exit 2
        ;;
    esac
  done

  # ROS 2本体やbuildツールは全構成の土台なので、他group指定時も自動で含める。
  SELECTED_GROUPS[base]=1
}

group_selected() {
  local group_name="$1"
  [[ "${SELECTED_GROUPS[${group_name}]:-0}" -eq 1 ]]
}

selected_groups_csv() {
  local groups=()
  local group_name=""

  for group_name in base simulation glim dev-test; do
    if group_selected "${group_name}"; then
      groups+=("${group_name}")
    fi
  done

  (IFS=','; printf '%s' "${groups[*]}")
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
    --profile=*)
      INSTALL_PROFILE="${1#*=}"
      ;;
    --profile)
      shift
      if [[ -z "${1:-}" || "${1:-}" == --* ]]; then
        echo "--profile requires a value." >&2
        exit 2
      fi
      INSTALL_PROFILE="$1"
      ;;
    --groups=*)
      INSTALL_GROUPS="${1#*=}"
      ;;
    --groups)
      shift
      if [[ -z "${1:-}" || "${1:-}" == --* ]]; then
        echo "--groups requires a comma-separated value." >&2
        exit 2
      fi
      INSTALL_GROUPS="$1"
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Run with --help to see available options." >&2
      exit 2
      ;;
  esac
  shift
done

if [[ -z "${INSTALL_GROUPS}" ]]; then
  INSTALL_GROUPS="$(profile_to_groups "${INSTALL_PROFILE}")"
fi
select_install_groups "${INSTALL_GROUPS}"

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
  bash "${SYSTEM_DEPENDENCIES_SCRIPT}" --groups "$(selected_groups_csv)"
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

install_base_packages() {
  local packages=(
    build-essential
    cmake
    git
    libeigen3-dev
    python3-colcon-common-extensions
    python3-rosdep
    python3-vcstool
    sudo
    "ros-${ROS_DISTRO}-ament-cmake"
    "ros-${ROS_DISTRO}-ament-cmake-auto"
    "ros-${ROS_DISTRO}-diagnostic-msgs"
    "ros-${ROS_DISTRO}-geometry-msgs"
    "ros-${ROS_DISTRO}-launch"
    "ros-${ROS_DISTRO}-launch-ros"
    "ros-${ROS_DISTRO}-nav-msgs"
    "ros-${ROS_DISTRO}-robot-state-publisher"
    "ros-${ROS_DISTRO}-ros-base"
    "ros-${ROS_DISTRO}-ros2launch"
    "ros-${ROS_DISTRO}-ros2run"
    "ros-${ROS_DISTRO}-ros2service"
    "ros-${ROS_DISTRO}-ros2topic"
    "ros-${ROS_DISTRO}-sensor-msgs"
    "ros-${ROS_DISTRO}-std-msgs"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-xacro"
  )

  # 実機・開発・シミュレーションの共通土台だけを導入する。
  install_apt_packages_if_missing "${packages[@]}"
}

install_simulation_packages() {
  local packages=(
    gazebo
    libapr1-dev
    libaprutil1-dev
    libboost-chrono-dev
    libboost-dev
    libgazebo-dev
    libprotobuf-dev
    protobuf-compiler
    "ros-${ROS_DISTRO}-gazebo-dev"
    "ros-${ROS_DISTRO}-gazebo-plugins"
    "ros-${ROS_DISTRO}-gazebo-ros"
    "ros-${ROS_DISTRO}-rviz2"
  )

  # Gazebo/Livoxシミュレーション専用の依存は選択時だけ導入する。
  install_apt_packages_if_missing "${packages[@]}"
}

install_glim_packages() {
  local packages=(
    libboost-all-dev
    libgomp1
    libpcl-dev
    libtbb-dev
    "ros-${ROS_DISTRO}-message-filters"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-rosbag2"
    "ros-${ROS_DISTRO}-tf2-sensor-msgs"
  )

  # glimと点群統合ノードが必要とする点群処理系依存をまとめる。
  install_apt_packages_if_missing "${packages[@]}"
}

install_dev_test_packages() {
  local packages=(
    clang-format
    "ros-${ROS_DISTRO}-ament-cmake-gtest"
    "ros-${ROS_DISTRO}-ament-lint-auto"
    "ros-${ROS_DISTRO}-ament-lint-common"
    "ros-${ROS_DISTRO}-launch-testing"
  )

  # 開発・テスト用途の補助依存は運用profileから外せるように分ける。
  install_apt_packages_if_missing "${packages[@]}"
}

install_system_dependencies() {
  group_selected base && install_base_packages
  group_selected simulation && install_simulation_packages
  group_selected glim && install_glim_packages
  group_selected dev-test && install_dev_test_packages

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

  # 外部repoは検証済みcommit SHAに固定し、default branchの更新でビルドや挙動が変わらないようにする。
  mkdir -p "$(dirname "${target_dir}")"
  git init "${target_dir}"
  git -C "${target_dir}" remote add origin "${repo_url}"
  git -C "${target_dir}" fetch --depth 1 origin "${expected_ref}"
  git -C "${target_dir}" checkout --detach FETCH_HEAD
}

patch_ros2_livox_simulation_repo() {
  local target_dir="$1"
  local marker_file="$2"

  # 非公式Livox Gazebo pluginは、本プロジェクトで必要な「CustomMsgとPointCloud2を別topicへ出す」「LiDAR原点基準の点群を出す」挙動を標準機能として持たない。
  # upstream forkを作ると追跡対象が増えるため、検証済みcommitに対して最小限の文字列patchを冪等適用し、変更範囲をこの関数に閉じ込める。
  # upstream側でtopic分離とray原点補正が正式対応された場合は、このpatchを削除してURDF側のplugin設定だけで運用する。
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

patch_gtsam_points_repo() {
  local target_dir="$1"
  local migration_header="${target_dir}/include/gtsam_points/util/gtsam_migration.hpp"

  if [[ ! -f "${migration_header}" ]]; then
    echo "Missing gtsam_points migration header: ${migration_header}" >&2
    return 1
  fi

  # gtsam_pointsの検証済みrevisionはUbuntu 22.04標準Boostでboost::none_tをconstexpr変数にできず、ビルドが失敗する。
  # APIや実行時挙動を変えず、該当変数のconstexpr指定だけを外す最小patchにする。upstreamがBoost 1.74互換修正を取り込んだrevisionへ更新したら削除する。
  sed -i 's/constexpr auto NoneValue = boost::none;/const auto NoneValue = boost::none;/' "${migration_header}"
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

  if [[ ! -d "${workspace_src}" ]]; then
    return 0
  fi

  if ! command -v rosdep >/dev/null 2>&1; then
    return 0
  fi

  ensure_rosdep_updated
  rosdep install --from-paths "${workspace_src}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

install_rosdeps_for_paths() {
  local paths=("$@")
  local existing_paths=()
  local path=""

  if ! command -v rosdep >/dev/null 2>&1; then
    return 0
  fi

  for path in "${paths[@]}"; do
    if [[ -d "${path}" ]]; then
      existing_paths+=("${path}")
    fi
  done

  if [[ "${#existing_paths[@]}" -eq 0 ]]; then
    return 0
  fi

  ensure_rosdep_updated
  rosdep install --from-paths "${existing_paths[@]}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

clone_simulation_repositories() {
  # Livox実機driver/SDKは公式repoを使う。simulation pluginだけはLivox MID-360非反復走査をGazebo Classicで再現するための非公式repoで、simulation group限定に閉じる。
  ensure_git_repo "${LIVOX_DRIVER_DIR}" "https://github.com/Livox-SDK/livox_ros_driver2.git" "${LIVOX_DRIVER_REF}"
  ensure_git_repo "${LIVOX_SIM_DIR}" "https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git" "${LIVOX_SIM_REF}"
  ensure_git_repo "${LIVOX_SDK_DIR}" "https://github.com/Livox-SDK/Livox-SDK2.git" "${LIVOX_SDK_REF}"
  patch_ros2_livox_simulation_repo "${LIVOX_SIM_DIR}" "${LIVOX_SIM_DIR}/.ai_ship_robot_patch_applied"
}

clone_glim_repositories() {
  # GLIM公式READMEで案内される構成を使い、GLIM本体・ROS 2 wrapper・点群factorライブラリを同じ検証済みrevisionに固定する。
  ensure_git_repo "${GTSAM_DIR}" "https://github.com/borglab/gtsam.git" "${GTSAM_REF}"
  ensure_git_repo "${GTSAM_POINTS_DIR}" "https://github.com/koide3/gtsam_points.git" "${GTSAM_POINTS_REF}"
  ensure_git_repo "${GLIM_DIR}" "https://github.com/koide3/glim.git" "${GLIM_REF}"
  ensure_git_repo "${GLIM_ROS2_DIR}" "https://github.com/koide3/glim_ros2.git" "${GLIM_ROS2_REF}"
  patch_gtsam_points_repo "${GTSAM_POINTS_DIR}"
}

clone_external_repositories() {
  group_selected simulation && clone_simulation_repositories
  group_selected glim && clone_glim_repositories
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

selected_third_party_paths() {
  local paths=()

  if group_selected simulation; then
    paths+=("${LIVOX_DRIVER_DIR}" "${LIVOX_SIM_DIR}")
  fi
  if group_selected glim; then
    paths+=("${GTSAM_DIR}" "${GTSAM_POINTS_DIR}" "${GLIM_DIR}" "${GLIM_ROS2_DIR}")
  fi

  printf '%s\n' "${paths[@]}"
}

build_third_party_paths() {
  local cxx_standard="$1"
  shift
  local paths=("$@")

  if [[ "${#paths[@]}" -eq 0 ]]; then
    return 0
  fi

  # 依存関係を既に満たしているrepo群だけを渡し、colconの並列configureで未install依存を参照しないようにする。
  colcon \
    --log-base "${THIRD_PARTY_WS}/log" \
    build \
    --base-paths "${paths[@]}" \
    --build-base "${THIRD_PARTY_WS}/build" \
    --install-base "${THIRD_PARTY_WS}/install" \
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

  if [[ -f "${THIRD_PARTY_WS}/install/setup.bash" ]]; then
    # 直前にinstallされたCMake configを後続repoのfind_packageから見つけられるようoverlayを更新する。
    set +u
    source "${THIRD_PARTY_WS}/install/setup.bash"
    set -u
  fi
}

build_third_party_workspace() {
  local third_party_paths=()

  mapfile -t third_party_paths < <(selected_third_party_paths)
  if [[ "${#third_party_paths[@]}" -eq 0 ]]; then
    return 0
  fi

  if group_selected simulation; then
    if [[ ! -d "${LIVOX_DRIVER_DIR}" ]]; then
      echo "Missing livox_ros_driver2 repository at ${LIVOX_DRIVER_DIR}" >&2
      return 1
    fi
    if [[ ! -d "${LIVOX_SIM_DIR}" ]]; then
      echo "Missing ros2_livox_simulation repository at ${LIVOX_SIM_DIR}" >&2
      return 1
    fi
  fi
  if group_selected glim; then
    if [[ ! -d "${GTSAM_DIR}" ]]; then
      echo "Missing GTSAM repository at ${GTSAM_DIR}" >&2
      return 1
    fi
    if [[ ! -d "${GTSAM_POINTS_DIR}" ]]; then
      echo "Missing gtsam_points repository at ${GTSAM_POINTS_DIR}" >&2
      return 1
    fi
    if [[ ! -d "${GLIM_DIR}" ]]; then
      echo "Missing glim repository at ${GLIM_DIR}" >&2
      return 1
    fi
    if [[ ! -d "${GLIM_ROS2_DIR}" ]]; then
      echo "Missing glim_ros2 repository at ${GLIM_ROS2_DIR}" >&2
      return 1
    fi
  fi

  if group_selected simulation; then
    build_third_party_paths 17 "${LIVOX_DRIVER_DIR}" "${LIVOX_SIM_DIR}"
  fi

  if group_selected glim; then
    build_third_party_paths 17 "${GTSAM_DIR}"
    build_third_party_paths 20 "${GTSAM_POINTS_DIR}"
    build_third_party_paths 20 "${GLIM_DIR}"
    build_third_party_paths 20 "${GLIM_ROS2_DIR}"
  fi
}

build_project_workspace() {
  # 通常のcolcon buildにして、Python/URDFのみの変更でも既存キャッシュを保持する。
  colcon --log-base "${ROS_WS}/log" build --base-paths "${ROS_WS}/src" --build-base "${ROS_WS}/build" --install-base "${ROS_WS}/install" --symlink-install
}

install_workspace() {
  source_ros2
  clone_external_repositories
  remove_stale_workspace_artifacts
  if group_selected simulation; then
    prepare_livox_simulation_build_environment
    install_livox_sdk2_if_needed
    prepare_livox_ros_driver2_manifest
  fi
  install_rosdeps_for_paths $(selected_third_party_paths)
  build_third_party_workspace

  # third_party overlayを重ねてからプロジェクトworkspaceのrosdep解決とビルドを行う。
  if [[ -f "${THIRD_PARTY_WS}/install/setup.bash" ]]; then
    set +u
    source "${THIRD_PARTY_WS}/install/setup.bash"
    set -u
  fi
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
