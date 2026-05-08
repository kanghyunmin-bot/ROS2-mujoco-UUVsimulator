#!/usr/bin/env bash
set -euo pipefail

DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${DIST_DIR}/../.." && pwd)"
OUT_DIR="${DIST_DIR}/out"
PACKAGE_NAME="uuv_sim_ubuntu22.04_dist2"
STAGE_DIR="${OUT_DIR}/${PACKAGE_NAME}"
ARCHIVE_PATH="${OUT_DIR}/${PACKAGE_NAME}.zip"
UUV_ZIP_PATH="${STAGE_DIR}/uuv_mujoco.zip"
ROSPKG_ZIP_PATH="${STAGE_DIR}/rospkg/kmu26_auv.zip"
PING360_MSG_ZIP_PATH="${STAGE_DIR}/rospkg/ping360_sonar_msgs.zip"
ALLOW_DIRTY=0

DOCSOURCE_FILES=(
  "analyze_april1_real_bags.py"
  "compare_closed_loop_april1_replay.py"
  "replay_april1_rc_override_closed_loop.py"
  "replay_april1_real_commands_in_mujoco.py"
  "run_closed_loop_april1_replay.sh"
  "run_uuv_param_autotune.py"
)

usage() {
  cat <<'USAGE'
Usage: ./dist2/ubuntu22.04/package_dist2.sh [options]

Package the current workspace into the dist2 Ubuntu 22.04 runtime zip.

Options:
  --out-dir PATH   Write package output under PATH instead of dist2/ubuntu22.04/out
  --allow-dirty    Allow packaging from a worktree with tracked local changes
  -h, --help       Show this help

By default this script refuses tracked local changes so release zips do not
accidentally include experiment-only simulator edits. Use package_from_github.sh
for the normal current-release path.
USAGE
}

require_file() {
  local path="$1"
  if [[ ! -f "${ROOT_DIR}/${path}" ]]; then
    echo "[dist2] required file missing: ${path}" >&2
    exit 1
  fi
}

require_dir() {
  local path="$1"
  if [[ ! -d "${ROOT_DIR}/${path}" ]]; then
    echo "[dist2] required directory missing: ${path}" >&2
    exit 1
  fi
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[dist2] command not found: $1" >&2
    exit 1
  }
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir)
      [[ $# -ge 2 ]] || { echo "[dist2] --out-dir requires a value" >&2; exit 2; }
      OUT_DIR="$2"
      shift 2
      ;;
    --allow-dirty)
      ALLOW_DIRTY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[dist2] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

STAGE_DIR="${OUT_DIR}/${PACKAGE_NAME}"
ARCHIVE_PATH="${OUT_DIR}/${PACKAGE_NAME}.zip"
UUV_ZIP_PATH="${STAGE_DIR}/uuv_mujoco.zip"
ROSPKG_ZIP_PATH="${STAGE_DIR}/rospkg/kmu26_auv.zip"
PING360_MSG_ZIP_PATH="${STAGE_DIR}/rospkg/ping360_sonar_msgs.zip"

source_remote="$(git -C "${ROOT_DIR}" config --get remote.origin.url 2>/dev/null || printf 'unknown')"
source_branch="$(git -C "${ROOT_DIR}" branch --show-current 2>/dev/null || printf 'unknown')"
source_commit="$(git -C "${ROOT_DIR}" rev-parse HEAD 2>/dev/null || printf 'unknown')"
source_status="$(git -C "${ROOT_DIR}" status --porcelain --untracked-files=no 2>/dev/null || true)"
source_submodules="$(git -C "${ROOT_DIR}" submodule status 2>/dev/null || true)"

if [[ -n "${source_status}" && "${ALLOW_DIRTY}" -eq 0 ]]; then
  echo "[dist2] refusing to package a dirty tracked worktree." >&2
  echo "[dist2] Use package_from_github.sh for a clean GitHub release package, or rerun with --allow-dirty intentionally." >&2
  echo "${source_status}" >&2
  exit 1
fi

require_cmd zip
require_file "uuv_control_gui.py"
require_file "run_control_gui.sh"
require_file "run_control_gui_ubuntu.sh"
require_file "cleanup_generated_artifacts.sh"
require_file "dist2/ubuntu22.04/install_uuv_sim_ubuntu22.sh"
require_file "dist2/ubuntu22.04/README_FIRST.md"
require_file "dist2/ubuntu22.04/README.md"
require_file "dist2/ubuntu22.04/DIST_GUIDE.md"
require_file "dist2/ubuntu22.04/PORTABILITY_AUDIT.md"
require_dir "uuv_mujoco/v2.2"
require_dir "rospkg/kmu26_auv"
require_dir "rospkg/ping360_sonar_msgs"

for file in "${DOCSOURCE_FILES[@]}"; do
  require_file "document/docsource/${file}"
done

rm -rf "${STAGE_DIR}" "${ARCHIVE_PATH}"
mkdir -p "${STAGE_DIR}/document/docsource" "${STAGE_DIR}/rospkg" "${OUT_DIR}"

cp "${DIST_DIR}/install_uuv_sim_ubuntu22.sh" "${STAGE_DIR}/install_uuv_sim_ubuntu22.sh"
cp "${DIST_DIR}/README_FIRST.md" "${STAGE_DIR}/README_FIRST.md"
cp "${DIST_DIR}/README.md" "${STAGE_DIR}/README.md"
cp "${DIST_DIR}/DIST_GUIDE.md" "${STAGE_DIR}/DIST_GUIDE.md"
cp "${DIST_DIR}/PORTABILITY_AUDIT.md" "${STAGE_DIR}/PORTABILITY_AUDIT.md"
cp "${ROOT_DIR}/uuv_control_gui.py" "${STAGE_DIR}/uuv_control_gui.py"
cp "${ROOT_DIR}/run_control_gui_ubuntu.sh" "${STAGE_DIR}/run_control_gui.sh"
cp "${ROOT_DIR}/run_control_gui_ubuntu.sh" "${STAGE_DIR}/run_control_gui_ubuntu.sh"
cp "${ROOT_DIR}/cleanup_generated_artifacts.sh" "${STAGE_DIR}/cleanup_generated_artifacts.sh"
cat > "${STAGE_DIR}/install_and_run.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/install_uuv_sim_ubuntu22.sh" --run-after-install "$@"
EOF
chmod +x \
  "${STAGE_DIR}/install_uuv_sim_ubuntu22.sh" \
  "${STAGE_DIR}/install_and_run.sh" \
  "${STAGE_DIR}/run_control_gui.sh" \
  "${STAGE_DIR}/run_control_gui_ubuntu.sh" \
  "${STAGE_DIR}/cleanup_generated_artifacts.sh"

for file in "${DOCSOURCE_FILES[@]}"; do
  cp "${ROOT_DIR}/document/docsource/${file}" "${STAGE_DIR}/document/docsource/${file}"
done
chmod +x "${STAGE_DIR}/document/docsource/"*.py "${STAGE_DIR}/document/docsource/"*.sh

(
  cd "${ROOT_DIR}"
  zip -qr "${UUV_ZIP_PATH}" uuv_mujoco \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig' \
    -x '*/logs/*' '*/logs' \
    -x '*/delete/*' '*/delete' \
    -x '*/.git/*'
)

(
  cd "${ROOT_DIR}/rospkg"
  zip -qr "${ROSPKG_ZIP_PATH}" kmu26_auv \
    -x '*/.git/*' '*/.git' \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig'
)

(
  cd "${ROOT_DIR}/rospkg"
  zip -qr "${PING360_MSG_ZIP_PATH}" ping360_sonar_msgs \
    -x '*/.git/*' '*/.git' \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig'
)

cat > "${STAGE_DIR}/RELEASE_MANIFEST.txt" <<EOF
${PACKAGE_NAME}

Generated: $(date -u '+%Y-%m-%dT%H:%M:%SZ')
Source remote: ${source_remote}
Source branch: ${source_branch}
Source commit: ${source_commit}
Tracked worktree dirty: $([[ -n "${source_status}" ]] && printf 'yes' || printf 'no')

Submodules:
${source_submodules:-none}

Top-level install:
  ./install_uuv_sim_ubuntu22.sh --noninteractive
  ./install_and_run.sh --noninteractive

Headless install/run:
  ./install_uuv_sim_ubuntu22.sh --noninteractive --run-headless
EOF

cat > "${STAGE_DIR}/BUNDLE_CONTENTS.txt" <<EOF
${PACKAGE_NAME}

Runtime package for Ubuntu 22.04.

Source:
- remote: ${source_remote}
- branch: ${source_branch}
- commit: ${source_commit}

Included:
- installer
- dist install guide
- portability audit
- install_and_run wrapper
- MuJoCo UUV runtime zip
- ROS2 helper package zip
- Ping360 SonarEcho message package zip
- control GUI
- Ubuntu-native GUI launcher
- minimal replay/autotune helper scripts

Not included:
- sample rosbag data
- full document tree
- ArduPilot checkout
- QGroundControl binary
EOF

(
  cd "${OUT_DIR}"
  zip -qr "${ARCHIVE_PATH##*/}" "${PACKAGE_NAME}"
)

(
  cd "${OUT_DIR}"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${ARCHIVE_PATH##*/}" > SHA256SUMS
  else
    shasum -a 256 "${ARCHIVE_PATH##*/}" > SHA256SUMS
  fi
)

echo "[dist2] stage:   ${STAGE_DIR}"
echo "[dist2] archive: ${ARCHIVE_PATH}"
echo "[dist2] sha256:  ${OUT_DIR}/SHA256SUMS"
