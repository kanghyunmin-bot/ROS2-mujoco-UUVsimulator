#!/usr/bin/env bash
set -euo pipefail

DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${DIST_DIR}/../.." && pwd)"
OUT_DIR="${DIST_DIR}/out/current-installer"
DIST_VERSION="${UUV_SIM_DIST_VERSION:-$(date +%Y.%m.%d-dist2)}"
TEMPLATE_ZIP=""
ALLOW_DIRTY=0

INNER_NAME="uuv_sim_current_ubuntu22.04"
DEB_PACKAGE="uuv-sim-current"

usage() {
  cat <<'USAGE'
Usage: ./dist2/ubuntu22.04/package_current_installer.sh [options]

Build the installable UUV Sim release zip:

  UUV_Sim_Install_and_Run_<version>.zip
    payload/uuv-sim-current_<version>_amd64.deb
    install_and_run_uuv_sim.sh
    Install and Run UUV Sim
    README.txt
    RELEASE_NOTES_<version>.txt
    SHA256SUMS.txt

Options:
  --version VALUE       Release/version string (default: YYYY.MM.DD-dist2)
  --out-dir PATH        Output directory (default: dist2/ubuntu22.04/out/current-installer)
  --template-zip PATH   Optional previous installer zip to reuse the ELF launcher
  --allow-dirty         Allow packaging from a dirty tracked worktree
  -h, --help            Show this help

Release zips and debs are generated artifacts. Upload them as GitHub Release
assets; do not commit them to the repository.
USAGE
}

fail() {
  echo "[current-installer] $*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "command not found: $1"
}

require_file() {
  local path="$1"
  [[ -f "${ROOT_DIR}/${path}" ]] || fail "required file missing: ${path}"
}

require_dir() {
  local path="$1"
  [[ -d "${ROOT_DIR}/${path}" ]] || fail "required directory missing: ${path}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      [[ $# -ge 2 ]] || fail "--version requires a value"
      DIST_VERSION="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || fail "--out-dir requires a value"
      OUT_DIR="$2"
      shift 2
      ;;
    --template-zip)
      [[ $# -ge 2 ]] || fail "--template-zip requires a value"
      TEMPLATE_ZIP="$2"
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
      fail "unknown option: $1"
      ;;
  esac
done

require_cmd git
require_cmd rsync
require_cmd zip
require_cmd unzip
require_cmd zipinfo
require_cmd python3
require_cmd dpkg-deb

if [[ -n "$TEMPLATE_ZIP" && ! -f "$TEMPLATE_ZIP" ]]; then
  fail "template zip missing: ${TEMPLATE_ZIP}"
fi

source_status="$(git -C "${ROOT_DIR}" status --porcelain --untracked-files=no 2>/dev/null || true)"
if [[ -n "$source_status" && "$ALLOW_DIRTY" -eq 0 ]]; then
  echo "[current-installer] refusing to package a dirty tracked worktree." >&2
  echo "$source_status" >&2
  echo "[current-installer] commit first, or rerun with --allow-dirty intentionally." >&2
  exit 1
fi

require_file "install_uuv_sim_current_ubuntu22.sh"
require_file "preflight_uuv_sim_current.sh"
require_file "verify_current_dist.sh"
require_file "install_and_run_web.sh"
require_file "run_control_gui.sh"
require_file "cleanup_generated_artifacts.sh"
require_file "uuv_control_gui.py"
require_dir "uuv_mujoco"
require_dir "rospkg/kmu26_auv"
require_dir "rospkg/dvl_msgs"
require_dir "rospkg/ping360_sonar_msgs"
require_file "rospkg/kmu26_auv/package.xml"
require_file "rospkg/dvl_msgs/package.xml"
require_file "rospkg/ping360_sonar_msgs/package.xml"

BUILD_DIR="$(mktemp -d /tmp/uuv_current_installer_XXXXXX)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

FINAL_NAME="UUV_Sim_Install_and_Run_${DIST_VERSION}"
FINAL_ZIP="${OUT_DIR}/${FINAL_NAME}.zip"
INNER_STAGE="${BUILD_DIR}/plain/${INNER_NAME}"
RUNTIME_STAGE="${BUILD_DIR}/runtime/uuv_mujoco"
DEB_ROOT="${BUILD_DIR}/debroot"
DEB_PATH="${BUILD_DIR}/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb"
OUTER_STAGE="${BUILD_DIR}/${FINAL_NAME}"

source_remote="$(git -C "${ROOT_DIR}" config --get remote.origin.url 2>/dev/null || printf 'unknown')"
source_branch="$(git -C "${ROOT_DIR}" branch --show-current 2>/dev/null || printf 'unknown')"
source_commit="$(git -C "${ROOT_DIR}" rev-parse HEAD 2>/dev/null || printf 'unknown')"

rm -rf "$OUT_DIR"
mkdir -p "$INNER_STAGE/rospkg" "$OUT_DIR" "$(dirname "$RUNTIME_STAGE")"

install -m 755 "${ROOT_DIR}/install_uuv_sim_current_ubuntu22.sh" "${INNER_STAGE}/install_uuv_sim_current_ubuntu22.sh"
install -m 755 "${ROOT_DIR}/preflight_uuv_sim_current.sh" "${INNER_STAGE}/preflight_uuv_sim_current.sh"
install -m 755 "${ROOT_DIR}/verify_current_dist.sh" "${INNER_STAGE}/verify_current_dist.sh"
install -m 755 "${ROOT_DIR}/install_and_run_web.sh" "${INNER_STAGE}/install_and_run_web.sh"
install -m 755 "${ROOT_DIR}/run_control_gui.sh" "${INNER_STAGE}/run_control_gui.sh"
install -m 755 "${ROOT_DIR}/cleanup_generated_artifacts.sh" "${INNER_STAGE}/cleanup_generated_artifacts.sh"
install -m 644 "${ROOT_DIR}/uuv_control_gui.py" "${INNER_STAGE}/uuv_control_gui.py"
if [[ -f "${ROOT_DIR}/README_FIRST_CURRENT.md" ]]; then
  install -m 644 "${ROOT_DIR}/README_FIRST_CURRENT.md" "${INNER_STAGE}/README_FIRST.md"
else
  install -m 644 "${DIST_DIR}/README_FIRST.md" "${INNER_STAGE}/README_FIRST.md"
fi
if [[ -f "${ROOT_DIR}/PORTABILITY_CHECKLIST_CURRENT.md" ]]; then
  install -m 644 "${ROOT_DIR}/PORTABILITY_CHECKLIST_CURRENT.md" "${INNER_STAGE}/PORTABILITY_CHECKLIST.md"
else
  install -m 644 "${DIST_DIR}/PORTABILITY_AUDIT.md" "${INNER_STAGE}/PORTABILITY_CHECKLIST.md"
fi
printf '%s\n' "$DIST_VERSION" > "${INNER_STAGE}/VERSION"

rsync -a \
  --exclude='**/__pycache__/***' \
  --exclude='**/.git/***' \
  --exclude='**/logs/***' \
  --exclude='**/private/***' \
  --exclude='**/delete/***' \
  --exclude='*.pyc' \
  --exclude='*.pyo' \
  --exclude='*.bak' \
  --exclude='*.bak_*' \
  --exclude='*~' \
  --exclude='*.orig' \
  --exclude='.DS_Store' \
  --exclude='MUJOCO_LOG.TXT' \
  "${ROOT_DIR}/uuv_mujoco/" "${RUNTIME_STAGE}/"
printf '%s\n' "$DIST_VERSION" > "${RUNTIME_STAGE}/.uuv_runtime_payload_version"
python3 - "${RUNTIME_STAGE}/RUNTIME_VERSION.json" "$DIST_VERSION" "$source_remote" "$source_branch" "$source_commit" <<'PY'
import json
import pathlib
import sys
from datetime import datetime, timezone

path = pathlib.Path(sys.argv[1])
version, remote, branch, commit = sys.argv[2:6]
data = {}
if path.exists():
    try:
        data = json.loads(path.read_text())
    except Exception:
        data = {}
data.update({
    "schema": 1,
    "updated_at": datetime.now(timezone.utc).date().isoformat(),
    "active_runtime": "uuv_mujoco/current",
    "backing_directory": "uuv_mujoco/v2.2",
    "status": "current",
    "freshness_status": "packaged-dist2",
    "active_runtime_label": f"current-{version}",
    "source_remote": remote,
    "source_branch": branch,
    "source_head": commit,
    "origin_uuv_sim_head": commit,
    "freshness_policy": f"Packaged from Git source for dist2 release {version}.",
    "primary_runner": "uuv_mujoco/current/run_uuv_mujoco.py",
    "compatibility_runner": "uuv_mujoco/current/run_urdf_full.py",
    "note": "dist2 installer package includes web GUI camera/joystick and MuJoCo Wayland viewer fixes.",
})
path.write_text(json.dumps(data, indent=2, sort_keys=False) + "\n")
PY

(
  cd "${BUILD_DIR}/runtime"
  zip -qr "${INNER_STAGE}/uuv_mujoco.zip" uuv_mujoco \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig' \
    -x '*/logs/*' '*/logs' \
    -x '*/delete/*' '*/delete' \
    -x '*/private/*' '*/private' \
    -x '*/.git/*' '*/.git' \
    -x '*/MUJOCO_LOG.TXT'
)

(
  cd "${ROOT_DIR}/rospkg"
  zip -qr "${INNER_STAGE}/rospkg/kmu26_auv.zip" kmu26_auv \
    -x '*/.git/*' '*/.git' \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig'
  zip -qr "${INNER_STAGE}/rospkg/dvl_msgs.zip" dvl_msgs \
    -x '*/.git/*' '*/.git' \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig'
  zip -qr "${INNER_STAGE}/rospkg/ping360_sonar_msgs.zip" ping360_sonar_msgs \
    -x '*/.git/*' '*/.git' \
    -x '*/.DS_Store' '.DS_Store' \
    -x '*/__pycache__/*' '*/__pycache__' \
    -x '*.pyc' '*.pyo' \
    -x '*.bak' '*.bak_*' '*~' '*.orig'
)

cat > "${INNER_STAGE}/BUNDLE_CONTENTS.txt" <<EOF
UUV simulator current runtime bundle
Version: ${DIST_VERSION}
Built: $(date -u +%Y-%m-%dT%H:%M:%SZ)

Included:
- install_uuv_sim_current_ubuntu22.sh
- preflight_uuv_sim_current.sh
- install_and_run_web.sh
- run_control_gui.sh / uuv_control_gui.py
- uuv_mujoco.zip
- rospkg/*.zip

This bundle includes the web GUI camera, joystick drag, telemetry refresh, and
MuJoCo Wayland/XWayland viewer fixes from the current Git source.
EOF

cat > "${INNER_STAGE}/RELEASE_MANIFEST.txt" <<EOF
release: ${DIST_VERSION}
package: ${INNER_NAME}
source_remote: ${source_remote}
source_branch: ${source_branch}
source_commit: ${source_commit}
built_at_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)

included_changes:
- Web GUI camera panel with stream on/off, zoom, and selectable resolution/FPS profiles.
- Stereo stream displays one camera view only to reduce GUI load.
- Camera profiles include 960x540@10Hz, 1280x720@5Hz, and 1280x720@10Hz.
- Joystick pointer handling supports click-and-drag, release centering, and pilot-input auto-enable.
- Vehicle summary, attitude, depth, and speed display refresh fixes.
- MuJoCo viewer defaults force XWayland/system GLFW on Wayland unless explicitly opted out.
EOF

(
  cd "${BUILD_DIR}/plain"
  zip -qr "${BUILD_DIR}/${INNER_NAME}.zip" "$INNER_NAME"
)

bash "${INNER_STAGE}/verify_current_dist.sh" "${BUILD_DIR}/${INNER_NAME}.zip"

mkdir -p \
  "${DEB_ROOT}/DEBIAN" \
  "${DEB_ROOT}/opt/uuv-sim-current" \
  "${DEB_ROOT}/usr/bin" \
  "${DEB_ROOT}/usr/share/applications" \
  "${DEB_ROOT}/usr/share/doc/uuv-sim-current"

cat > "${DEB_ROOT}/DEBIAN/control" <<EOF
Package: ${DEB_PACKAGE}
Version: ${DIST_VERSION}
Section: science
Priority: optional
Architecture: amd64
Maintainer: UUV Simulator Maintainers <robot@localhost>
Depends: bash, coreutils, python3, unzip, curl, git, sudo, ca-certificates, gnupg, lsb-release, xz-utils
Recommends: xwayland, libdecor-0-0, libglfw3, libgl1, libegl1, libgl1-mesa-dri, libosmesa6, libfuse2, python3-venv, python3-pip, python3-dev, python3-tk, build-essential, cmake, pkg-config
Description: Current UUV MuJoCo ArduSub SITL simulator distribution
 Installs the current UUV MuJoCo simulator distribution payload and helper
 commands. Run uuv-sim-current-install to install dependencies, clone
 ArduPilot, build bundled ROS 2 helper packages, and start the web GUI.
EOF

cat > "${DEB_ROOT}/DEBIAN/postinst" <<'EOF'
#!/usr/bin/env bash
set -e
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database /usr/share/applications >/dev/null 2>&1 || true
fi
cat <<'MSG'

uuv-sim-current installed.

Terminal commands:
  uuv-sim-current-web
  uuv-sim-current-install --noninteractive
  uuv-sim-current-preflight

MSG
exit 0
EOF

cat > "${DEB_ROOT}/DEBIAN/postrm" <<'EOF'
#!/usr/bin/env bash
set -e
exit 0
EOF

printf '%s\n' "$DIST_VERSION" > "${DEB_ROOT}/opt/uuv-sim-current/VERSION"
install -m 644 "${BUILD_DIR}/${INNER_NAME}.zip" "${DEB_ROOT}/opt/uuv-sim-current/${INNER_NAME}.zip"
install -m 755 "${ROOT_DIR}/preflight_uuv_sim_current.sh" "${DEB_ROOT}/opt/uuv-sim-current/preflight_uuv_sim_current.sh"
install -m 644 "${INNER_STAGE}/README_FIRST.md" "${DEB_ROOT}/usr/share/doc/uuv-sim-current/README_FIRST.md"
install -m 644 "${INNER_STAGE}/RELEASE_MANIFEST.txt" "${DEB_ROOT}/usr/share/doc/uuv-sim-current/RELEASE_MANIFEST.txt"

cat > "${DEB_ROOT}/usr/bin/uuv-sim-current-install" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
PAYLOAD="/opt/uuv-sim-current/uuv_sim_current_ubuntu22.04.zip"
TARGET="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"
[[ -f "$PAYLOAD" ]] || { echo "[uuv-sim-current] payload missing: $PAYLOAD" >&2; exit 1; }
mkdir -p "$TARGET"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT
unzip -q "$PAYLOAD" -d "$TMP_DIR"
PKG_DIR="${TMP_DIR}/uuv_sim_current_ubuntu22.04"
cp -a "${PKG_DIR}/." "$TARGET/"
cd "$TARGET"
exec ./install_uuv_sim_current_ubuntu22.sh --install-root "$TARGET" "$@"
EOF

cat > "${DEB_ROOT}/usr/bin/uuv-sim-current-preflight" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
TARGET="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"
PREFLIGHT="/opt/uuv-sim-current/preflight_uuv_sim_current.sh"
has_install_root=0
for arg in "$@"; do
  [[ "$arg" == "--install-root" ]] && has_install_root=1 && break
done
if [[ "$has_install_root" -eq 1 ]]; then
  exec "$PREFLIGHT" "$@"
else
  exec "$PREFLIGHT" --install-root "$TARGET" "$@"
fi
EOF

cat > "${DEB_ROOT}/usr/bin/uuv-sim-current-web" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
TARGET="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"
ENV_FILE="${TARGET}/.uuv_mujoco_env.sh"
TARGET_VERSION_FILE="${TARGET}/.uuv_sim_current_version"
PAYLOAD_VERSION_FILE="/opt/uuv-sim-current/VERSION"

read_version() {
  local path="$1"
  [[ -f "$path" ]] || return 0
  tr -d '[:space:]' <"$path"
}

payload_version="$(read_version "$PAYLOAD_VERSION_FILE")"
target_version="$(read_version "$TARGET_VERSION_FILE")"
if [[ ! -f "$ENV_FILE" ]]; then
  uuv-sim-current-install --noninteractive
elif [[ -n "$payload_version" && "$target_version" != "$payload_version" ]]; then
  uuv-sim-current-install --noninteractive --force-reextract --skip-apt --skip-qgc --skip-ardupilot --skip-rospkg-build --skip-python-env
fi
source "$ENV_FILE"
exec "${TARGET}/run_control_gui.sh" --web "$@"
EOF

cat > "${DEB_ROOT}/usr/bin/uuv-sim-current-gui" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
TARGET="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"
ENV_FILE="${TARGET}/.uuv_mujoco_env.sh"
[[ -f "$ENV_FILE" ]] || uuv-sim-current-install --noninteractive
source "$ENV_FILE"
exec "${TARGET}/run_control_gui.sh" "$@"
EOF

cat > "${DEB_ROOT}/usr/bin/uuv-sim-current-headless" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
TARGET="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"
ENV_FILE="${TARGET}/.uuv_mujoco_env.sh"
[[ -f "$ENV_FILE" ]] || uuv-sim-current-install --noninteractive
source "$ENV_FILE"
cd "${UUV_MUJOCO_DIR}/current"
exec ./start_sitl_mujoco_mj311.sh -- --headless "$@"
EOF

cat > "${DEB_ROOT}/usr/share/applications/uuv-sim-current-web.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=UUV Sim Current Web GUI
Comment=Install and run the UUV simulator web GUI
Exec=uuv-sim-current-web
Terminal=true
Categories=Science;Robotics;
EOF

find "$DEB_ROOT" -type d -exec chmod 755 {} +
find "${DEB_ROOT}/DEBIAN" -type f -exec chmod 755 {} +
find "${DEB_ROOT}/usr/bin" -type f -exec chmod 755 {} +
dpkg-deb --build --root-owner-group "$DEB_ROOT" "$DEB_PATH"

mkdir -p "${OUTER_STAGE}/payload"
install -m 644 "$DEB_PATH" "${OUTER_STAGE}/payload/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb"

cat > "${OUTER_STAGE}/install_and_run_uuv_sim.sh" <<EOF
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
DEB="\${SCRIPT_DIR}/payload/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb"
[[ -f "\$DEB" ]] || { echo "[uuv-install] deb missing: \$DEB" >&2; exit 1; }

run_root() {
  if [[ "\$(id -u)" -eq 0 ]]; then
    "\$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "\$@"
  elif command -v pkexec >/dev/null 2>&1; then
    pkexec "\$@"
  else
    echo "[uuv-install] need sudo or pkexec to install the deb package" >&2
    exit 1
  fi
}

if command -v apt-get >/dev/null 2>&1; then
  run_root apt-get install -y "\$DEB"
else
  run_root dpkg -i "\$DEB"
fi

exec uuv-sim-current-web
EOF
chmod 755 "${OUTER_STAGE}/install_and_run_uuv_sim.sh"

if [[ -n "$TEMPLATE_ZIP" ]]; then
  mkdir -p "${BUILD_DIR}/template"
  unzip -q "$TEMPLATE_ZIP" '*/Install and Run UUV Sim' -d "${BUILD_DIR}/template" || true
  template_launcher="$(find "${BUILD_DIR}/template" -name 'Install and Run UUV Sim' -type f | head -1 || true)"
  if [[ -n "$template_launcher" ]]; then
    install -m 755 "$template_launcher" "${OUTER_STAGE}/Install and Run UUV Sim"
  fi
fi
if [[ ! -f "${OUTER_STAGE}/Install and Run UUV Sim" ]]; then
  cp "${OUTER_STAGE}/install_and_run_uuv_sim.sh" "${OUTER_STAGE}/Install and Run UUV Sim"
  chmod 755 "${OUTER_STAGE}/Install and Run UUV Sim"
fi

cat > "${OUTER_STAGE}/README.txt" <<EOF
UUV Sim Install and Run (${DIST_VERSION})

Ubuntu 22.04 amd64 installer bundle. Run:

  ./install_and_run_uuv_sim.sh

or execute "Install and Run UUV Sim".

The installer installs payload/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb, then
starts the web GUI. Existing ~/uuv_sim_current installs are refreshed when the
payload version differs.
EOF

cat > "${OUTER_STAGE}/RELEASE_NOTES_${DIST_VERSION}.txt" <<EOF
UUV Sim ${DIST_VERSION}

Source:
- remote: ${source_remote}
- branch: ${source_branch}
- commit: ${source_commit}

Highlights:
- Web GUI camera panel with stream on/off, zoom, and selectable profiles.
- 1280x720 camera profile support, including 10 Hz.
- Joystick click-and-drag handling and pilot-input auto-enable.
- Vehicle summary, attitude, depth, and speed refresh fixes.
- MuJoCo viewer window defaults for Wayland sessions via XWayland/system GLFW.
EOF

(
  cd "$OUTER_STAGE"
  sha256sum \
    "payload/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb" \
    "Install and Run UUV Sim" \
    install_and_run_uuv_sim.sh \
    README.txt \
    "RELEASE_NOTES_${DIST_VERSION}.txt" > SHA256SUMS.txt
)

(
  cd "$BUILD_DIR"
  zip -qr "$FINAL_ZIP" "$FINAL_NAME"
)

install -m 644 "${BUILD_DIR}/${INNER_NAME}.zip" "${OUT_DIR}/${INNER_NAME}.zip"
install -m 644 "$DEB_PATH" "${OUT_DIR}/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb"

echo "[current-installer] final zip: ${FINAL_ZIP}"
echo "[current-installer] deb:       ${OUT_DIR}/${DEB_PACKAGE}_${DIST_VERSION}_amd64.deb"
echo "[current-installer] payload:   ${OUT_DIR}/${INNER_NAME}.zip"
sha256sum "$FINAL_ZIP"
