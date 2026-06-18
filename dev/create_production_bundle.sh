#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_ROOT_DEFAULT="${WORKSPACE_ROOT}/outputs/deploy"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
LIVOX_SDK_INSTALL_DIR="${SYSTEM_INSTALL_ROOT}/vendor/livox_sdk2/install"
THIRD_PARTY_WS_INSTALL_DIR="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install"
OUTPUT_ROOT="${OUTPUT_ROOT_DEFAULT}"
NAME_SUFFIX=""
CHECK_DIRTY=false

usage() {
  cat <<'EOF'
Usage: bash dev/create_production_bundle.sh [OPTIONS]

本番環境向けの zip 配布バンドルを生成します。
- app 本体の source / install / scripts を同梱します
- /opt/ai_ship_robot 配下の build 済み third_party install 成果物を同梱します
- 展開先で使う deploy script と manifest を自動生成します

Options:
  --output-dir PATH   バンドル出力先ディレクトリを指定します。
  --name-suffix TEXT  zip ファイル名に任意の suffix を付与します。
  --check-dirty       git worktree に未commit変更がある場合は失敗します。
  -h, --help          この help を表示します。
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"

  if [[ -z "${value}" || "${value}" == --* ]]; then
    echo "${option} requires a value." >&2
    exit 2
  fi

  printf '%s' "${value}"
}

require_command() {
  local command_name="$1"

  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Required command not found: ${command_name}" >&2
    exit 1
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --output-dir)
        OUTPUT_ROOT="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      --name-suffix)
        NAME_SUFFIX="$(require_value "$1" "${2:-}")"
        shift 2
        ;;
      --check-dirty)
        CHECK_DIRTY=true
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done
}

require_existing_directory() {
  local directory_path="$1"

  if [[ ! -d "${directory_path}" ]]; then
    echo "Required directory not found: ${directory_path}" >&2
    exit 1
  fi
}

require_existing_file() {
  local file_path="$1"

  if [[ ! -f "${file_path}" ]]; then
    echo "Required file not found: ${file_path}" >&2
    exit 1
  fi
}

check_required_inputs() {
  # 配布対象の主要入力と /opt 配下の成果物を先に確認し、作成途中で欠落に気づく状態を防ぐ。
  require_existing_file "${WORKSPACE_ROOT}/README.md"
  require_existing_file "${WORKSPACE_ROOT}/install/install.sh"
  require_existing_file "${WORKSPACE_ROOT}/install/setup.sh"
  require_existing_file "${WORKSPACE_ROOT}/scripts/run_slam.sh"
  require_existing_directory "${WORKSPACE_ROOT}/ros2_ws/src"
  require_existing_directory "${LIVOX_SDK_INSTALL_DIR}"
  require_existing_directory "${THIRD_PARTY_WS_INSTALL_DIR}"
}

check_git_state() {
  local status_output=""

  if ! git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    return 0
  fi

  status_output="$(git -C "${WORKSPACE_ROOT}" status --short)"
  if [[ -n "${status_output}" && "${CHECK_DIRTY}" == "true" ]]; then
    echo "Refusing to create bundle from a dirty worktree because --check-dirty was specified." >&2
    echo "${status_output}" >&2
    exit 1
  fi
}

copy_tree() {
  local source_path="$1"
  local destination_path="$2"

  mkdir -p "${destination_path}"
  cp -a "${source_path}/." "${destination_path}/"
}

write_deploy_script() {
  local deploy_script_path="$1"

  # 展開先では bundled third_party を /opt 配下へ配置してから既存導線を呼べるように専用 script を同梱する。
  cat > "${deploy_script_path}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_ROOT="${BUNDLE_ROOT}/app"
ARTIFACT_ROOT="${BUNDLE_ROOT}/third_party_artifacts"
SYSTEM_INSTALL_ROOT="/opt/ai_ship_robot"
LIVOX_DEST_DIR="${SYSTEM_INSTALL_ROOT}/vendor/livox_sdk2/install"
THIRD_PARTY_DEST_DIR="${SYSTEM_INSTALL_ROOT}/ros_underlay/${ROS_DISTRO}/third_party_ws/install"

usage() {
  cat <<'INNER_EOF'
Usage: bash deploy/install_bundled_third_party.sh

Bundled third_party 成果物を /opt/ai_ship_robot へ配置したうえで、
既存の install/install.sh と install/setup.sh を順に実行します。
INNER_EOF
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

require_existing_directory() {
  local directory_path="$1"

  if [[ ! -d "${directory_path}" ]]; then
    echo "Required directory not found: ${directory_path}" >&2
    exit 1
  fi
}

copy_artifact_tree() {
  local source_path="$1"
  local destination_path="$2"

  # install 成果物は配置先を空にせず上書きコピーし、部分更新でも必要ファイルを欠かさないようにする。
  ${SUDO} install -d "${destination_path}"
  ${SUDO} cp -a "${source_path}/." "${destination_path}/"
}

write_ld_config() {
  local config_path="/etc/ld.so.conf.d/ai_ship_robot.conf"
  local temp_config=""

  # Livox SDK の共有ライブラリ探索先を明示し、起動時の LD_LIBRARY_PATH 依存を減らす。
  temp_config="$(mktemp)"
  {
    printf '%s\n' "${SYSTEM_INSTALL_ROOT}/vendor/livox_sdk2/install/lib"
    if [[ -d "${SYSTEM_INSTALL_ROOT}/vendor/livox_sdk2/install/lib64" ]]; then
      printf '%s\n' "${SYSTEM_INSTALL_ROOT}/vendor/livox_sdk2/install/lib64"
    fi
  } > "${temp_config}"
  ${SUDO} install -m 0644 "${temp_config}" "${config_path}"
  rm -f "${temp_config}"
  ${SUDO} ldconfig
}

require_existing_directory "${ARTIFACT_ROOT}/livox_sdk2/install"
require_existing_directory "${ARTIFACT_ROOT}/third_party_ws/install"

copy_artifact_tree "${ARTIFACT_ROOT}/livox_sdk2/install" "${LIVOX_DEST_DIR}"
copy_artifact_tree "${ARTIFACT_ROOT}/third_party_ws/install" "${THIRD_PARTY_DEST_DIR}"
write_ld_config

bash "${APP_ROOT}/install/install.sh"
bash "${APP_ROOT}/install/setup.sh"
EOF
  chmod +x "${deploy_script_path}"
}

write_deploy_guide() {
  local deploy_guide_path="$1"
  local bundle_root_name="$2"

  # 展開後の手順を配布物内へ固定し、現地で参照する情報源を一本化する。
  cat > "${deploy_guide_path}" <<EOF
# ${bundle_root_name} 配備手順

1. zip を展開する
2. 展開先へ移動する
3. 次を実行する

\`\`\`bash
bash deploy/install_bundled_third_party.sh
\`\`\`

\`install/install.sh\` はインターネット経由で apt / ROS 依存を導入します。
bundled third_party 成果物は上記 script が \`/opt/ai_ship_robot\` へ配置します。
EOF
}

write_manifest() {
  local manifest_path="$1"
  local created_at="$2"
  local git_commit="$3"
  local git_branch="$4"
  local bundle_root_name="$5"

  # 配布物の出自と同梱済み成果物の配置元を残し、現地調査時に bundle の中身を追跡しやすくする。
  cat > "${manifest_path}" <<EOF
{
  "project": "ai_ship_robot",
  "bundle_root": "${bundle_root_name}",
  "created_at": "${created_at}",
  "git_commit": "${git_commit}",
  "git_branch": "${git_branch}",
  "ros_distro": "${ROS_DISTRO}",
  "workspace_root": "${WORKSPACE_ROOT}",
  "third_party_artifacts": {
    "livox_sdk_install": "${LIVOX_SDK_INSTALL_DIR}",
    "third_party_ws_install": "${THIRD_PARTY_WS_INSTALL_DIR}"
  }
}
EOF
}

create_bundle() {
  local created_at="$(date --iso-8601=seconds)"
  local timestamp="$(date +%Y%m%d_%H%M%S)"
  local git_commit="$(git -C "${WORKSPACE_ROOT}" rev-parse --short HEAD 2>/dev/null || printf 'unknown')"
  local git_branch="$(git -C "${WORKSPACE_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || printf 'unknown')"
  local sanitized_suffix=""
  local bundle_root_name="ai_ship_robot_prod_${timestamp}_${git_commit}"
  local staging_root=""
  local bundle_root=""
  local zip_path=""

  if [[ -n "${NAME_SUFFIX}" ]]; then
    sanitized_suffix="$(printf '%s' "${NAME_SUFFIX}" | tr ' /' '__')"
    bundle_root_name+="_${sanitized_suffix}"
  fi

  staging_root="$(mktemp -d)"
  bundle_root="${staging_root}/${bundle_root_name}"
  zip_path="${OUTPUT_ROOT}/${bundle_root_name}.zip"

  trap "rm -rf '${staging_root}'" EXIT

  # 配布物の階層を固定し、展開先 script が相対パスだけで app と artifact を見つけられるようにする。
  mkdir -p "${bundle_root}/app" "${bundle_root}/deploy" "${bundle_root}/third_party_artifacts/livox_sdk2" "${bundle_root}/third_party_artifacts/third_party_ws"

  copy_tree "${WORKSPACE_ROOT}/install" "${bundle_root}/app/install"
  copy_tree "${WORKSPACE_ROOT}/scripts" "${bundle_root}/app/scripts"
  copy_tree "${WORKSPACE_ROOT}/ros2_ws/src" "${bundle_root}/app/ros2_ws/src"
  cp -a "${WORKSPACE_ROOT}/README.md" "${bundle_root}/app/README.md"
  copy_tree "${LIVOX_SDK_INSTALL_DIR}" "${bundle_root}/third_party_artifacts/livox_sdk2/install"
  copy_tree "${THIRD_PARTY_WS_INSTALL_DIR}" "${bundle_root}/third_party_artifacts/third_party_ws/install"

  write_deploy_script "${bundle_root}/deploy/install_bundled_third_party.sh"
  write_deploy_guide "${bundle_root}/deploy/DEPLOY.md" "${bundle_root_name}"
  write_manifest "${bundle_root}/deployment_manifest.json" "${created_at}" "${git_commit}" "${git_branch}" "${bundle_root_name}"

  mkdir -p "${OUTPUT_ROOT}"

  # zip には bundle root ごと格納し、受け取り側で展開先ディレクトリ名が一意になるようにする。
  (
    cd "${staging_root}"
    zip -qr "${zip_path}" "${bundle_root_name}"
  )

  sha256sum "${zip_path}" > "${zip_path}.sha256"
  echo "Created bundle: ${zip_path}"
  echo "Checksum: ${zip_path}.sha256"
}

parse_args "$@"
require_command git
require_command zip
require_command sha256sum
check_required_inputs
check_git_state
create_bundle
