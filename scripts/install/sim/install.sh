#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
DEBIAN_FRONTEND="${DEBIAN_FRONTEND:-noninteractive}"
APT_PRIMARY_MIRROR="${APT_PRIMARY_MIRROR:-}"
APT_PORTS_MIRROR="${APT_PORTS_MIRROR:-}"
APT_SECURITY_MIRROR="${APT_SECURITY_MIRROR:-}"
APT_FORCE_IPV4="${APT_FORCE_IPV4:-true}"
APT_RETRIES="${APT_RETRIES:-3}"
APT_HTTP_TIMEOUT="${APT_HTTP_TIMEOUT:-20}"
APT_HTTPS_TIMEOUT="${APT_HTTPS_TIMEOUT:-${APT_HTTP_TIMEOUT}}"
APT_UPDATED=0
export DEBIAN_FRONTEND

usage() {
  cat <<'EOF'
Usage: bash scripts/install/sim/install.sh

simulation向けの system 依存導入を実行します。
- Gazebo / RViz / Livox simulation 用 apt 依存導入
- ROS 2 repository 設定
- rosdep 初期化

事前に `bash scripts/install/install.sh` を実行してください。
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

run_with_safe_apt_locale() {
  # locales導入前でもapt maintainer scriptが未生成localeを参照しないよう、安全なUTF-8 localeでapt系処理を実行する。
  ${SUDO} env DEBIAN_FRONTEND="${DEBIAN_FRONTEND}" LANG=C.UTF-8 LC_ALL=C.UTF-8 "$@"
}

mark_apt_indexes_stale() {
  APT_UPDATED=0
}

collect_missing_packages() {
  local -n missing_packages_ref="$1"
  shift
  local package_name=""
  local package_status=""
  local normalized_name=""
  declare -A installed_packages=()
  missing_packages_ref=()
  # dpkg-queryをまとめて呼び、再実行時の不要なプロセス起動を減らす。
  while IFS=$'\t' read -r package_name package_status; do
    if [[ "${package_status}" == "install ok installed" ]]; then
      normalized_name="${package_name%%:*}"
      installed_packages["${package_name}"]=1
      installed_packages["${normalized_name}"]=1
    fi
  done < <(dpkg-query -W -f='${Package}\t${Status}\n' "$@" 2>/dev/null || true)
  for package_name in "$@"; do
    if [[ "${installed_packages[${package_name}]:-0}" -ne 1 ]]; then missing_packages_ref+=("${package_name}"); fi
  done
}

apt_update_if_needed() {
  if [[ "${APT_UPDATED}" -eq 1 ]]; then return 0; fi
  run_with_safe_apt_locale apt-get update
  APT_UPDATED=1
}

install_apt_packages_if_missing() {
  local missing_packages=()
  # simulation追加分でも未導入packageだけに絞り、再実行時の待ち時間を減らす。
  collect_missing_packages missing_packages "$@"
  if [[ "${#missing_packages[@]}" -eq 0 ]]; then return 0; fi
  apt_update_if_needed
  run_with_safe_apt_locale apt-get install -y --no-install-recommends "${missing_packages[@]}"
}

write_apt_network_config() {
  local apt_config_path="/etc/apt/apt.conf.d/99ai-ship-robot-network"
  local temp_config="$(mktemp)"
  # 不安定な回線でもaptが長時間停止しにくいよう、通信設定を1箇所へ固定する。
  {
    printf 'Acquire::Retries "%s";\n' "${APT_RETRIES}"
    printf 'Acquire::http::Timeout "%s";\n' "${APT_HTTP_TIMEOUT}"
    printf 'Acquire::https::Timeout "%s";\n' "${APT_HTTPS_TIMEOUT}"
    if [[ "${APT_FORCE_IPV4}" == "true" ]]; then printf 'Acquire::ForceIPv4 "true";\n'; fi
  } > "${temp_config}"
  ${SUDO} install -m 0644 "${temp_config}" "${apt_config_path}"
  rm -f "${temp_config}"
}

backup_file_once() {
  local file_path="$1"
  local backup_path="${file_path}.ai_ship_robot.bak"
  if [[ -f "${backup_path}" || ! -f "${file_path}" ]]; then return 0; fi
  ${SUDO} cp "${file_path}" "${backup_path}"
}

replace_apt_source_url() {
  local current_url="$1"
  local replacement_url="$2"
  local temp_sources=""
  if [[ -z "${replacement_url}" || ! -f /etc/apt/sources.list ]]; then return 0; fi
  if ! grep -Fq "${current_url}" /etc/apt/sources.list; then return 0; fi
  temp_sources="$(mktemp)"
  sed "s#${current_url}#${replacement_url%/}#g" /etc/apt/sources.list > "${temp_sources}"
  if cmp -s "${temp_sources}" /etc/apt/sources.list; then rm -f "${temp_sources}"; return 0; fi
  backup_file_once "/etc/apt/sources.list"
  ${SUDO} install -m 0644 "${temp_sources}" /etc/apt/sources.list
  rm -f "${temp_sources}"
  mark_apt_indexes_stale
}

configure_ubuntu_apt_mirrors() {
  # install.shと同じmirror設定入口を使い、開発環境でも通信条件を揃える。
  replace_apt_source_url "http://archive.ubuntu.com/ubuntu" "${APT_PRIMARY_MIRROR}"
  replace_apt_source_url "https://archive.ubuntu.com/ubuntu" "${APT_PRIMARY_MIRROR}"
  replace_apt_source_url "http://ports.ubuntu.com/ubuntu-ports" "${APT_PORTS_MIRROR}"
  replace_apt_source_url "https://ports.ubuntu.com/ubuntu-ports" "${APT_PORTS_MIRROR}"
  replace_apt_source_url "http://security.ubuntu.com/ubuntu" "${APT_SECURITY_MIRROR}"
  replace_apt_source_url "https://security.ubuntu.com/ubuntu" "${APT_SECURITY_MIRROR}"
}

apt_source_has_component() {
  local component_name="$1"
  grep -Eqs "^[[:space:]]*deb .*[[:space:]]${component_name}([[:space:]]|$)" /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null || grep -Eqs "^[[:space:]]*Components: .*[[:space:]]?${component_name}([[:space:]]|$)" /etc/apt/sources.list.d/*.sources 2>/dev/null
}

enable_apt_component_in_sources_list() {
  local component_name="$1"
  local sources_file="/etc/apt/sources.list"
  local temp_sources=""
  local line=""
  local changed=0
  if apt_source_has_component "${component_name}"; then return 0; fi
  if [[ ! -f "${sources_file}" ]]; then echo "Missing ${sources_file}; cannot enable apt component: ${component_name}" >&2; return 1; fi
  temp_sources="$(mktemp)"
  # 追加repository toolを増やさず、既存deb行へ必要componentだけを追記する。
  while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ "${line}" =~ ^[[:space:]]*deb[[:space:]] ]] && [[ "${line}" =~ [[:space:]]main([[:space:]]|$) ]] && [[ ! "${line}" =~ [[:space:]]${component_name}([[:space:]]|$) ]]; then
      printf '%s %s\n' "${line}" "${component_name}" >> "${temp_sources}"
      changed=1
      continue
    fi
    printf '%s\n' "${line}" >> "${temp_sources}"
  done < "${sources_file}"
  if [[ "${changed}" -eq 0 ]]; then rm -f "${temp_sources}"; echo "No editable apt source line found for component: ${component_name}" >&2; return 1; fi
  backup_file_once "${sources_file}"
  ${SUDO} install -m 0644 "${temp_sources}" "${sources_file}"
  rm -f "${temp_sources}"
  mark_apt_indexes_stale
}

ensure_rosdep_initialized() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then ${SUDO} rosdep init || true; fi
}

configure_ros2_apt_repository() {
  local prerequisite_packages=(ca-certificates curl gnupg locales tzdata)
  local ros_keyring_path="/etc/apt/keyrings/ros-archive-keyring.gpg"
  local ros_repo_file="/etc/apt/sources.list.d/ros2.list"
  local ros_repo_line="deb [arch=$(dpkg --print-architecture) signed-by=${ros_keyring_path}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main"
  # simulation用install単体でもrepository設定が不足しないよう、共通入口を内包する。
  write_apt_network_config
  configure_ubuntu_apt_mirrors
  install_apt_packages_if_missing "${prerequisite_packages[@]}"
  if ! apt_source_has_component universe; then enable_apt_component_in_sources_list universe; fi
  ${SUDO} install -m 0755 -d /etc/apt/keyrings
  if [[ ! -f "${ros_keyring_path}" ]]; then curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | ${SUDO} tee "${ros_keyring_path}" >/dev/null; fi
  if [[ ! -f "${ros_repo_file}" ]] || [[ "$(<"${ros_repo_file}")" != "${ros_repo_line}" ]]; then printf '%s\n' "${ros_repo_line}" | ${SUDO} tee "${ros_repo_file}" >/dev/null; mark_apt_indexes_stale; fi
}

install_simulation_packages() {
  local packages=(gazebo libapr1-dev libaprutil1-dev libboost-chrono-dev libboost-dev libgazebo-dev libprotobuf-dev protobuf-compiler "ros-${ROS_DISTRO}-gazebo-dev" "ros-${ROS_DISTRO}-gazebo-plugins" "ros-${ROS_DISTRO}-gazebo-ros" "ros-${ROS_DISTRO}-rviz2")
  # Gazebo ClassicとLivox simulation専用の依存だけを追加導入し、本番導線と責務を分ける。
  install_apt_packages_if_missing "${packages[@]}"
}

configure_ros2_apt_repository
install_simulation_packages
ensure_rosdep_initialized
