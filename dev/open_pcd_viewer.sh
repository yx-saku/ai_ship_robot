#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUTS_DIR="${WORKSPACE_ROOT}/outputs"

usage() {
  cat <<'EOF'
Usage: bash dev/open_pcd_viewer.sh [PCD_PATH]

PCD_PATHを指定した場合はそのファイルをpcl_viewerで開きます。
省略した場合はoutputsディレクトリ内の最新PCDファイルを探して開きます。

Options:
  -h, --help    Show this help.
EOF
}

find_latest_pcd_file() {
  if [[ ! -d "${OUTPUTS_DIR}" ]]; then
    echo "outputs directory does not exist: ${OUTPUTS_DIR}" >&2
    return 1
  fi

  local latest_file=""
  local pcd_file=""

  # outputs配下のPCDだけを対象にし、更新時刻が最も新しい地図を既定の閲覧対象にする。
  shopt -s nullglob globstar
  for pcd_file in "${OUTPUTS_DIR}"/**/*.pcd; do
    if [[ -z "${latest_file}" || "${pcd_file}" -nt "${latest_file}" ]]; then
      latest_file="${pcd_file}"
    fi
  done
  shopt -u nullglob globstar

  if [[ -z "${latest_file}" ]]; then
    echo "No PCD files found under: ${OUTPUTS_DIR}" >&2
    return 1
  fi

  printf '%s\n' "${latest_file}"
}

resolve_pcd_path() {
  local pcd_path="${1:-}"

  if [[ -z "${pcd_path}" ]]; then
    find_latest_pcd_file
    return $?
  fi

  # 相対パス指定を呼び出し位置に依存させず、ワークスペースrootからの指定として解決する。
  if [[ "${pcd_path}" != /* ]]; then
    pcd_path="${WORKSPACE_ROOT}/${pcd_path}"
  fi

  if [[ ! -f "${pcd_path}" ]]; then
    echo "PCD file does not exist: ${pcd_path}" >&2
    return 1
  fi
  if [[ "${pcd_path}" != *.pcd ]]; then
    echo "File extension is not .pcd: ${pcd_path}" >&2
    return 1
  fi

  printf '%s\n' "${pcd_path}"
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi
  if [[ "$#" -gt 1 ]]; then
    echo "Too many arguments." >&2
    usage >&2
    exit 2
  fi

  local pcd_path=""
  pcd_path="$(resolve_pcd_path "${1:-}")"

  if ! command -v pcl_viewer >/dev/null 2>&1; then
    echo "pcl_viewer command was not found. Install pcl-tools or source the environment that provides it." >&2
    exit 1
  fi

  # 選択したPCDを明示してからexecし、pcl_viewer終了コードをそのまま呼び出し元へ返す。
  echo "Opening PCD file with pcl_viewer: ${pcd_path}" >&2
  exec pcl_viewer "${pcd_path}"
}

main "$@"
