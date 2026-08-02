#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION="$(tr -d '[:space:]' <"${ROOT_DIR}/VERSION")"
DEB_PATH="${1:-${ROOT_DIR}/documentary/release/builds/${VERSION}/kmu-auv-simulator_${VERSION}_amd64.deb}"

fail() { echo "[verify-release] FAIL: $*" >&2; exit 1; }
pass() { echo "[verify-release] PASS: $*"; }

for cmd in dpkg-deb unzip zipinfo python3; do
  command -v "$cmd" >/dev/null 2>&1 || fail "command missing: $cmd"
done
[[ -s "$DEB_PATH" ]] || fail "DEB missing: $DEB_PATH"

TMP_DIR="$(mktemp -d /tmp/kmu-auv-verify.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT
dpkg-deb -x "$DEB_PATH" "$TMP_DIR/root"
dpkg-deb -e "$DEB_PATH" "$TMP_DIR/control"
ROOT="$TMP_DIR/root"
BUNDLE="$ROOT/opt/kmu-auv-simulator/bundle"

required=(
  opt/kmu-auv-simulator/VERSION
  opt/kmu-auv-simulator/bundle/VERSION
  opt/kmu-auv-simulator/bundle/uuv_mujoco.zip
  opt/kmu-auv-simulator/bundle/sim/current/assets/yolo/best.pt
  usr/bin/kmu-auv-simulator
  usr/bin/kmu-auv-simulator-installer
  usr/bin/kmu-auv-simulator-uninstall
  usr/share/applications/kmu-auv-simulator.desktop
  usr/share/applications/kmu-auv-simulator-installer.desktop
  usr/share/applications/kmu-auv-simulator-uninstall.desktop
  usr/share/icons/hicolor/scalable/apps/kmu-auv-simulator.svg
)
for path in "${required[@]}"; do
  [[ -s "$ROOT/$path" ]] || fail "required DEB path missing: $path"
done
if find "$BUNDLE" -type f ! -perm -004 -print -quit | grep -q .; then
  fail "bundle contains a file that the installing user cannot read"
fi
if find "$BUNDLE" -type d ! -perm -005 -print -quit | grep -q .; then
  fail "bundle contains a directory that the installing user cannot traverse"
fi
pass "native DEB layout"

bash -n "$ROOT/usr/bin/kmu-auv-simulator" \
  "$ROOT/usr/bin/kmu-auv-simulator-installer" \
  "$ROOT/usr/bin/kmu-auv-simulator-uninstall" \
  "$TMP_DIR/control/postinst" \
  "$TMP_DIR/control/prerm" \
  "$TMP_DIR/control/postrm" \
  "$BUNDLE/install_uuv_sim_current_ubuntu22.sh" \
  "$BUNDLE/preflight_uuv_sim_current.sh"
pass "shell entry points parse"

unzip -tq "$BUNDLE/uuv_mujoco.zip" >/dev/null
for marker in \
  sim/current/run_uuv_mujoco.py \
  sim/current/scenes/tank_current_scene.xml \
  sim/current/bridge/ros2_mission_contract.py \
  sim/current/gui/sim_stack_launch_command.py
do
  unzip -l "$BUNDLE/uuv_mujoco.zip" "$marker" | grep -F "$marker" >/dev/null || \
    fail "runtime marker missing: $marker"
done
pass "MuJoCo runtime payload"

expected=(
  dvl_msgs auv_dvl_a50_msg ping360_sonar_msgs auv_msg auv
  audio_common_msgs audio_common audio_capture hydrophone_ctrl
  auv_buoy_vision_control auv_lane_vision_control
  kmu26_auv_surface_buoy_mission auv_web_gui
  auv_pinger_homing robot_localization
)
mkdir -p "$TMP_DIR/ros"
for archive in "$BUNDLE"/rospkg/*.zip; do
  unzip -q "$archive" -d "$TMP_DIR/ros"
done
for package in "${expected[@]}"; do
  if ! find "$TMP_DIR/ros" -name package.xml -type f -print0 | \
    xargs -0 grep -l "<name>${package}</name>" | grep -q .; then
    fail "ROS package missing: $package"
  fi
done
[[ -s "$TMP_DIR/ros/kmu26_auv_msg/msg/CollectorState.msg" ]] || \
  fail "CollectorState.msg missing"
[[ -s "$TMP_DIR/ros/kmu26_auv_surface_buoy_mission/src/surface_buoy_mission_node.cpp" ]] || \
  fail "surface mission node missing"
[[ -s "$TMP_DIR/ros/kmu26_auv_buoy_vision_control/models/best.pt" ]] || \
  fail "packaged detector model missing"
pass "current ROS autonomy payload"

bad_pattern='(^|/)(\.git|__pycache__|build|install|log|logs|private)(/|$)|\.py[co]$|\.(bag|db3|mcap)$|(^|/)core(\.[0-9]+)?$'
if zipinfo -1 "$BUNDLE/uuv_mujoco.zip" | grep -E "$bad_pattern" >/dev/null; then
  fail "runtime archive contains generated, VCS, build or log artifacts"
fi
for archive in "$BUNDLE"/rospkg/*.zip; do
  if zipinfo -1 "$archive" | grep -E "$bad_pattern" >/dev/null; then
    fail "ROS archive contains generated, VCS, build or log artifacts: $(basename "$archive")"
  fi
done
pass "archive hygiene"

unzip -p "$BUNDLE/uuv_mujoco.zip" sim/current/gui/sim_stack_launch_command.py \
  | grep -F 'DEFAULT_CAMERA_PRESET_ID = "competition_fixed"' >/dev/null || \
  fail "fixed camera contract missing"
grep -Fq 'auv_lane_vision_control' "$BUNDLE/install_uuv_sim_current_ubuntu22.sh" || \
  fail "installer does not build lane/surface autonomy"
grep -Fq 'kmu26_auv_surface_buoy_mission' "$BUNDLE/install_uuv_sim_current_ubuntu22.sh" || \
  fail "installer does not build the separate surface package"
if grep -Fq 'run "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh"' \
  "$BUNDLE/install_uuv_sim_current_ubuntu22.sh"; then
  fail "installer still executes the ArduSub 4.1.2 legacy Ubuntu prerequisite helper"
fi
grep -Fq 'python-is-python3' "$BUNDLE/install_uuv_sim_current_ubuntu22.sh" || \
  fail "installer does not provide the ArduSub Python 3 command contract"
pass "competition installer contracts"

UNINSTALL_HOME="$TMP_DIR/uninstall-home"
mkdir -p "$UNINSTALL_HOME/data/kmu-auv-simulator/current/sim/current" \
  "$UNINSTALL_HOME/state/kmu-auv-simulator" \
  "$UNINSTALL_HOME/config/kmu-auv-simulator" \
  "$UNINSTALL_HOME/cache/kmu-auv-simulator" \
  "$UNINSTALL_HOME/config/QGroundControl.org" \
  "$UNINSTALL_HOME/cache/QGroundControl.org" \
  "$UNINSTALL_HOME/venvs/uuv_mujoco"
printf '%s\n' "$VERSION" >"$UNINSTALL_HOME/data/kmu-auv-simulator/current/.uuv_sim_current_version"
printf 'home = /usr/bin\n' >"$UNINSTALL_HOME/venvs/uuv_mujoco/pyvenv.cfg"
: >"$UNINSTALL_HOME/state/kmu-auv-simulator/qgc-managed-by-kmu-auv"
env HOME="$UNINSTALL_HOME" \
  XDG_DATA_HOME="$UNINSTALL_HOME/data" \
  XDG_STATE_HOME="$UNINSTALL_HOME/state" \
  XDG_CONFIG_HOME="$UNINSTALL_HOME/config" \
  XDG_CACHE_HOME="$UNINSTALL_HOME/cache" \
  UUV_SIM_VENV_ROOT="$UNINSTALL_HOME/venvs/uuv_mujoco" \
  "$ROOT/usr/bin/kmu-auv-simulator-uninstall" --yes --keep-package
[[ ! -e "$UNINSTALL_HOME/data/kmu-auv-simulator/current" ]] || fail "uninstaller left workspace"
[[ ! -e "$UNINSTALL_HOME/venvs/uuv_mujoco" ]] || fail "uninstaller left virtualenv"
[[ ! -e "$UNINSTALL_HOME/state/kmu-auv-simulator" ]] || fail "uninstaller left state"
[[ ! -e "$UNINSTALL_HOME/config/QGroundControl.org" ]] || fail "uninstaller left managed QGC config"
[[ ! -e "$UNINSTALL_HOME/cache/QGroundControl.org" ]] || fail "uninstaller left managed QGC cache"
pass "complete uninstaller removes managed user data"

echo "[verify-release] package OK: $DEB_PATH"
