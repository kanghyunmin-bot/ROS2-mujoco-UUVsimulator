#!/usr/bin/env bash
set -euo pipefail

DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${DIST_DIR}/../.." && pwd)"
REPO_URL="${REPO_URL:-$(git -C "${ROOT_DIR}" config --get remote.origin.url 2>/dev/null || printf 'https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git')}"
BRANCH="${BRANCH:-uuv_sim}"
OUT_DIR="${OUT_DIR:-${DIST_DIR}/out/latest}"
KEEP_TMP=0

usage() {
  cat <<'USAGE'
Usage: ./dist2/ubuntu22.04/package_from_github.sh [options]

Build the dist2 runtime zip from a clean GitHub branch checkout, using the
local dist2 packaging tools. This is the preferred release path because it
keeps local uncommitted simulator experiments out of the uploaded zip.

Options:
  --repo-url URL   Git repository URL (default: current origin)
  --branch NAME    Git branch to package (default: uuv_sim)
  --out-dir PATH   Output directory (default: dist2/ubuntu22.04/out/latest)
  --keep-tmp       Keep the temporary clone for inspection
  -h, --help       Show this help
USAGE
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[dist2-github] command not found: $1" >&2
    exit 1
  }
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-url)
      [[ $# -ge 2 ]] || { echo "[dist2-github] --repo-url requires a value" >&2; exit 2; }
      REPO_URL="$2"
      shift 2
      ;;
    --branch)
      [[ $# -ge 2 ]] || { echo "[dist2-github] --branch requires a value" >&2; exit 2; }
      BRANCH="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { echo "[dist2-github] --out-dir requires a value" >&2; exit 2; }
      OUT_DIR="$2"
      shift 2
      ;;
    --keep-tmp)
      KEEP_TMP=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[dist2-github] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

require_cmd git
require_cmd zip
require_cmd unzip
require_cmd zipinfo
require_cmd python3

TMP_DIR="$(mktemp -d /tmp/uuv_dist2_github_XXXXXX)"
if [[ "$KEEP_TMP" -eq 0 ]]; then
  trap 'rm -rf "${TMP_DIR}"' EXIT
else
  echo "[dist2-github] keeping temp dir: ${TMP_DIR}"
fi

echo "[dist2-github] clone: ${REPO_URL} (${BRANCH})"
git clone -b "$BRANCH" --single-branch "$REPO_URL" "${TMP_DIR}/repo"
git -C "${TMP_DIR}/repo" submodule update --init rospkg/kmu26_auv

mkdir -p "${TMP_DIR}/repo/dist2"
cp -R "${DIST_DIR}" "${TMP_DIR}/repo/dist2/ubuntu22.04"
rm -rf "${TMP_DIR}/repo/dist2/ubuntu22.04/out"

(
  cd "${TMP_DIR}/repo"
  ./dist2/ubuntu22.04/package_dist2.sh
  ./dist2/ubuntu22.04/verify_package.sh
)

commit="$(git -C "${TMP_DIR}/repo" rev-parse --short HEAD)"
full_commit="$(git -C "${TMP_DIR}/repo" rev-parse HEAD)"
safe_branch="${BRANCH//\//-}"
date_tag="$(date '+%Y%m%d')"
archive_name="uuv_sim_ubuntu22.04_dist2_${safe_branch}_${commit}_${date_tag}.zip"

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
cp "${TMP_DIR}/repo/dist2/ubuntu22.04/out/uuv_sim_ubuntu22.04_dist2.zip" "${OUT_DIR}/${archive_name}"
cp "${TMP_DIR}/repo/dist2/ubuntu22.04/out/uuv_sim_ubuntu22.04_dist2/RELEASE_MANIFEST.txt" "${OUT_DIR}/RELEASE_MANIFEST.txt"

{
  echo "GitHub dist2 package"
  echo
  echo "Repo: ${REPO_URL}"
  echo "Branch: ${BRANCH}"
  echo "Commit: ${full_commit}"
  echo "Archive: ${archive_name}"
} > "${OUT_DIR}/README_UPLOAD.txt"

(
  cd "${OUT_DIR}"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${archive_name}" > SHA256SUMS
  else
    shasum -a 256 "${archive_name}" > SHA256SUMS
  fi
)

echo "[dist2-github] archive: ${OUT_DIR}/${archive_name}"
echo "[dist2-github] sha256:  ${OUT_DIR}/SHA256SUMS"
cat "${OUT_DIR}/SHA256SUMS"
