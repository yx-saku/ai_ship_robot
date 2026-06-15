#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AITRAN_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
AI_SHIP_ROBOT_OPT_ROOT="${AI_SHIP_ROBOT_OPT_ROOT:-/opt/ai_ship_robot}"
THIRD_PARTY_WS="${AI_SHIP_ROBOT_OPT_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws"
THIRD_PARTY_SRC_DIR="${THIRD_PARTY_WS}/src"
THIRD_PARTY_BUILD_DIR="${THIRD_PARTY_WS}/build"
THIRD_PARTY_INSTALL_DIR="${THIRD_PARTY_WS}/install"
THIRD_PARTY_LOG_DIR="${THIRD_PARTY_WS}/log"
LIVOX_DRIVER_DIR="${THIRD_PARTY_SRC_DIR}/livox_ros_driver2"
GTSAM_DIR="${THIRD_PARTY_SRC_DIR}/gtsam"
LIO_SAM_DIR="${THIRD_PARTY_SRC_DIR}/lio_sam_mid360_ros2"
LIO_SAM_OVERRIDE_DIR="${AITRAN_ROOT}/third_party_overrides/lio_sam_mid360_ros2"
AUTO_RCCAR_INDOOR_DIR="${THIRD_PARTY_SRC_DIR}/autoRCcar_indoor"
AUTO_RCCAR_INTERFACES_DIR="${AUTO_RCCAR_INDOOR_DIR}/ros2/src/autorccar_interfaces"
LIVOX_SDK_ROOT="${AI_SHIP_ROBOT_OPT_ROOT}/vendor/livox_sdk2"
LIVOX_SDK_SRC_DIR="${LIVOX_SDK_ROOT}/src"
LIVOX_SDK_BUILD_DIR="${LIVOX_SDK_ROOT}/build"
LIVOX_SDK_PREFIX="${LIVOX_SDK_ROOT}/install"
LIVOX_DRIVER_REF="13eb05e4e6dd7a765b934d0c5fd6236676a57b49"
LIVOX_SDK_REF="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
GTSAM_REF="4f66a491ffc83cf092d0d818b11dc35135521612"
LIO_SAM_REF="066a3e44c6a8e3cb1bd1bdd49b2cb2365711a213"
AUTO_RCCAR_INDOOR_REF="bc06c0176c4f6f1fdead58377045cea473a0b74c"
ROSDEP_UPDATE_MAX_AGE_SECONDS="${ROSDEP_UPDATE_MAX_AGE_SECONDS:-86400}"
ROSDEP_UPDATED=0

usage() {
  cat <<'EOF'
Usage: bash aitran/scripts/install/install_third_party.sh

実機・simulation共通の third_party を system 側へ導入します。
- Livox-SDK2 を /opt/ai_ship_robot/vendor/livox_sdk2 に導入
- livox_ros_driver2 / GTSAM / LIO-SAM を ROS underlay として導入
- underlay install: /opt/ai_ship_robot/ros_underlay/${ROS_DISTRO}/third_party_ws/install
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
    echo "Missing /opt/ros/${ROS_DISTRO}/setup.bash. Run bash aitran/scripts/install/install.sh first." >&2
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

  # /opt配下の作業領域はsystem配置だが、非root実行時はbuildできる所有者で作成する。
  if [[ "$(id -u)" -eq 0 ]]; then
    mkdir -p "${directory_path}"
  else
    ${SUDO} install -d -o "$(id -u)" -g "$(id -g)" "${directory_path}"
    ${SUDO} chown -R "$(id -u):$(id -g)" "${directory_path}"
  fi
}

prepare_install_roots() {
  # ROS underlay workspace と vendor SDK の source/build/install を成果物の近くへまとめる。
  ensure_owned_directory "${THIRD_PARTY_SRC_DIR}"
  ensure_owned_directory "${THIRD_PARTY_BUILD_DIR}"
  ensure_owned_directory "${THIRD_PARTY_INSTALL_DIR}"
  ensure_owned_directory "${THIRD_PARTY_LOG_DIR}"
  ensure_owned_directory "${LIVOX_SDK_ROOT}"
  ensure_owned_directory "${LIVOX_SDK_BUILD_DIR}"
  ensure_owned_directory "${LIVOX_SDK_PREFIX}"
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
    current_ref="$(git -c safe.directory="${target_dir}" -C "${target_dir}" rev-parse HEAD)"
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

  # 外部repoは検証済みcommitへ固定し、default branch更新で挙動が変わらないようにする。
  mkdir -p "$(dirname "${target_dir}")"
  git init "${target_dir}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" remote add origin "${repo_url}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" fetch --depth 1 origin "${expected_ref}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" checkout --detach FETCH_HEAD
}

ensure_sparse_git_repo() {
  local target_dir="$1"
  local repo_url="$2"
  local expected_ref="$3"
  local sparse_path="$4"
  local current_ref=""

  if [[ -d "${target_dir}/.git" ]]; then
    current_ref="$(git -c safe.directory="${target_dir}" -C "${target_dir}" rev-parse HEAD)"
    if [[ "${current_ref}" != "${expected_ref}" ]]; then
      echo "Unexpected repository revision at ${target_dir}" >&2
      echo "  expected: ${expected_ref}" >&2
      echo "  current : ${current_ref}" >&2
      echo "Remove the directory or update the pinned ref after validating the new revision." >&2
      return 1
    fi
    if [[ -e "${target_dir}/${sparse_path}" ]]; then
      return 0
    fi
    git -c safe.directory="${target_dir}" -C "${target_dir}" sparse-checkout set "${sparse_path}"
    return 0
  fi

  if [[ -e "${target_dir}" ]]; then
    if [[ ! -d "${target_dir}" ]] || ! directory_is_empty "${target_dir}"; then
      echo "Existing path is not a git repository: ${target_dir}" >&2
      return 1
    fi
  fi

  # 大きなupstream workspaceから必要なinterface packageだけを取得し、LIO-SAM本体は無改修でbuildする。
  mkdir -p "$(dirname "${target_dir}")"
  git init "${target_dir}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" remote add origin "${repo_url}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" sparse-checkout init --cone
  git -c safe.directory="${target_dir}" -C "${target_dir}" sparse-checkout set "${sparse_path}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" fetch --depth 1 origin "${expected_ref}"
  git -c safe.directory="${target_dir}" -C "${target_dir}" checkout --detach FETCH_HEAD
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

copy_lio_sam_mid360_overrides() {
  local current_ref=""
  local relative_path=""
  local source_path=""
  local destination_path=""
  local override_files=(
    "README.md"
    "LICENSE"
    "include/lio_sam/utility.hpp"
    "src/imageProjection.cpp"
    "src/featureExtraction.cpp"
    "src/mapOptmization.cpp"
    "src/imuPreintegration.cpp"
  )

  if [[ ! -d "${LIO_SAM_OVERRIDE_DIR}" ]]; then
    echo "Missing LIO-SAM override directory: ${LIO_SAM_OVERRIDE_DIR}" >&2
    return 1
  fi

  current_ref="$(git -c safe.directory="${LIO_SAM_DIR}" -C "${LIO_SAM_DIR}" rev-parse HEAD)"
  if [[ "${current_ref}" != "${LIO_SAM_REF}" ]]; then
    echo "Refusing to copy LIO-SAM overrides to an unexpected revision." >&2
    echo "  expected: ${LIO_SAM_REF}" >&2
    echo "  current : ${current_ref}" >&2
    return 1
  fi

  # 固定commitのupstream fileを、workspaceで管理するレビュー済みoverrideへ置き換える。
  for relative_path in "${override_files[@]}"; do
    source_path="${LIO_SAM_OVERRIDE_DIR}/${relative_path}"
    destination_path="${LIO_SAM_DIR}/${relative_path}"
    if [[ ! -f "${source_path}" ]]; then
      echo "Missing LIO-SAM override file: ${source_path}" >&2
      return 1
    fi
    if [[ -f "${destination_path}" ]] && cmp -s "${source_path}" "${destination_path}"; then
      continue
    fi

    if [[ -w "${destination_path}" ]]; then
      install -m 0644 "${source_path}" "${destination_path}"
    else
      ${SUDO} install -m 0644 "${source_path}" "${destination_path}"
    fi
  done
}

prepare_lio_sam_build_cache() {
  local cache_file="${THIRD_PARTY_BUILD_DIR}/lio_sam/CMakeCache.txt"
  local expected_home="CMAKE_HOME_DIRECTORY:INTERNAL=${LIO_SAM_DIR}"

  if [[ ! -f "${cache_file}" ]]; then
    return 0
  fi

  # clone先ディレクトリ名変更後はCMake cacheが旧source pathを指すため、LIO-SAM分だけ再configureさせる。
  if ! grep -Fxq "${expected_home}" "${cache_file}"; then
    rm -rf "${THIRD_PARTY_BUILD_DIR}/lio_sam"
  fi
}

clone_common_repositories() {
  # MID-360実機とsimulationで共通利用するrepoをsystem側の固定workspaceへ取得する。
  ensure_git_repo "${LIVOX_SDK_SRC_DIR}" "https://github.com/Livox-SDK/Livox-SDK2.git" "${LIVOX_SDK_REF}"
  ensure_git_repo "${LIVOX_DRIVER_DIR}" "https://github.com/Livox-SDK/livox_ros_driver2.git" "${LIVOX_DRIVER_REF}"
  ensure_git_repo "${GTSAM_DIR}" "https://github.com/borglab/gtsam.git" "${GTSAM_REF}"
  ensure_sparse_git_repo "${AUTO_RCCAR_INDOOR_DIR}" "https://github.com/UV-Lab/autoRCcar_indoor.git" "${AUTO_RCCAR_INDOOR_REF}" "ros2/src/autorccar_interfaces"
  ensure_git_repo "${LIO_SAM_DIR}" "https://github.com/UV-Lab/LIO-SAM_MID360_ROS2.git" "${LIO_SAM_REF}"
  prepare_livox_ros_driver2_manifest
  copy_lio_sam_mid360_overrides
  prepare_lio_sam_build_cache
}

prepend_env_path() {
  local variable_name="$1"
  local path_value="$2"
  local current_value="${!variable_name:-}"

  if [[ -z "${path_value}" ]]; then
    return 0
  fi

  # CMakeや実行時loaderへvendor SDKを先に見せ、/usr/localへ混ぜずにdriverを解決する。
  case ":${current_value}:" in
    *":${path_value}:"*) ;;
    *) export "${variable_name}=${path_value}${current_value:+:${current_value}}" ;;
  esac
}

configure_livox_sdk_environment() {
  # colcon build と実行時の両方で /opt 配下のSDKを優先して参照できるようにする。
  prepend_env_path CMAKE_PREFIX_PATH "${LIVOX_SDK_PREFIX}"
  prepend_env_path LD_LIBRARY_PATH "${LIVOX_SDK_PREFIX}/lib"
  if [[ -d "${LIVOX_SDK_PREFIX}/lib64" ]]; then
    prepend_env_path LD_LIBRARY_PATH "${LIVOX_SDK_PREFIX}/lib64"
  fi
  if [[ -d "${LIVOX_SDK_PREFIX}/lib/pkgconfig" ]]; then
    prepend_env_path PKG_CONFIG_PATH "${LIVOX_SDK_PREFIX}/lib/pkgconfig"
  fi
}

write_livox_sdk_ld_config() {
  local config_path="/etc/ld.so.conf.d/ai_ship_robot.conf"
  local temp_config=""

  temp_config="$(mktemp)"
  {
    printf '%s\n' "${LIVOX_SDK_PREFIX}/lib"
    if [[ -d "${LIVOX_SDK_PREFIX}/lib64" ]]; then
      printf '%s\n' "${LIVOX_SDK_PREFIX}/lib64"
    fi
  } > "${temp_config}"

  # vendor SDKの共有ライブラリをsystem loaderへ登録し、起動script側の環境変数依存を減らす。
  ${SUDO} install -m 0644 "${temp_config}" "${config_path}"
  rm -f "${temp_config}"
  ${SUDO} ldconfig
}

install_livox_sdk2_if_needed() {
  local livox_sdk_cxx_flags="${CMAKE_CXX_FLAGS:-}"

  configure_livox_sdk_environment
  if [[ -f "${LIVOX_SDK_PREFIX}/lib/liblivox_lidar_sdk_shared.so" || -f "${LIVOX_SDK_PREFIX}/lib64/liblivox_lidar_sdk_shared.so" ]]; then
    write_livox_sdk_ld_config
    return 0
  fi
  if [[ ! -d "${LIVOX_SDK_SRC_DIR}" ]]; then
    echo "Missing Livox-SDK2 repository at ${LIVOX_SDK_SRC_DIR}" >&2
    return 1
  fi

  # SDKソースを汚さずに大量のpragma警告だけを抑制し、ログの可読性を保つ。
  if [[ " ${livox_sdk_cxx_flags} " != *" -Wno-pragmas "* ]]; then
    livox_sdk_cxx_flags="${livox_sdk_cxx_flags:+${livox_sdk_cxx_flags} }-Wno-pragmas"
  fi

  # Livox driverが要求する共有SDKを /opt 配下のvendor prefixへ導入する。
  cmake -S "${LIVOX_SDK_SRC_DIR}" -B "${LIVOX_SDK_BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${LIVOX_SDK_PREFIX}" -DCMAKE_CXX_FLAGS="${livox_sdk_cxx_flags}"
  cmake --build "${LIVOX_SDK_BUILD_DIR}" --parallel
  cmake --install "${LIVOX_SDK_BUILD_DIR}"
  write_livox_sdk_ld_config
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

  # rosdep cacheが十分新しい場合は更新を省略し、Docker build時のネットワーク待ちを減らす。
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

  # third_party build直前だけrosdep indexを更新し、複数script実行時の重複更新を抑える。
  rosdep update
  mkdir -p "${HOME}/.ros/rosdep"
  date +%s > "${HOME}/.ros/rosdep/.ai_ship_robot_updated_at"
  ROSDEP_UPDATED=1
}

selected_common_ros_paths() {
  printf '%s\n' "${LIVOX_DRIVER_DIR}" "${GTSAM_DIR}" "${AUTO_RCCAR_INTERFACES_DIR}" "${LIO_SAM_DIR}"
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

  # system underlayへ入れる外部ROS packageのapt依存をbuild前にまとめて解決する。
  ensure_rosdep_updated
  rosdep install --from-paths "${existing_paths[@]}" --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
}

source_third_party_underlay_if_exists() {
  if [[ ! -f "${THIRD_PARTY_INSTALL_DIR}/setup.bash" ]]; then
    return 0
  fi

  # colcon workspaceを段階buildするため、既に入ったpackageを次のpackage解決に使う。
  set +u
  source "${THIRD_PARTY_INSTALL_DIR}/setup.bash"
  set -u
}

build_third_party_paths() {
  local cxx_standard="$1"
  shift
  local paths=("$@")

  if [[ "${#paths[@]}" -eq 0 ]]; then return 0; fi
  configure_livox_sdk_environment
  source_third_party_underlay_if_exists

  # 依存順に分割してbuildし、colcon並列configure時の未解決参照を避ける。
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
      -DCMAKE_PREFIX_PATH="${LIVOX_SDK_PREFIX};${THIRD_PARTY_INSTALL_DIR}" \
      -DCMAKE_CXX_STANDARD="${cxx_standard}" \
      -DROS_EDITION=ROS2 \
      -DDISTRO_ROS="${ROS_DISTRO}"
  source_third_party_underlay_if_exists
}

build_common_underlay_workspace() {
  if [[ ! -d "${LIVOX_DRIVER_DIR}" || ! -d "${GTSAM_DIR}" || ! -d "${AUTO_RCCAR_INTERFACES_DIR}" || ! -d "${LIO_SAM_DIR}" ]]; then
    echo "Missing common third-party repositories under ${THIRD_PARTY_SRC_DIR}" >&2
    return 1
  fi

  # 実機driverとLIO-SAMを同じinstall-baseへ積み、プロジェクトworkspaceの下地にする。
  build_third_party_paths 17 "${LIVOX_DRIVER_DIR}"
  build_third_party_paths 17 "${GTSAM_DIR}"
  build_third_party_paths 17 "${AUTO_RCCAR_INTERFACES_DIR}"
  build_third_party_paths 17 "${LIO_SAM_DIR}"
}

install_common_third_party() {
  source_ros2
  prepare_install_roots
  clone_common_repositories
  install_livox_sdk2_if_needed
  install_rosdeps_for_paths $(selected_common_ros_paths)
  build_common_underlay_workspace
  echo "Installed common third-party underlay: ${THIRD_PARTY_INSTALL_DIR}"
}

require_ros2
install_common_third_party
