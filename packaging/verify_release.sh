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
  usr/share/applications/kmu-auv-simulator.desktop
  usr/share/applications/kmu-auv-simulator-installer.desktop
  usr/share/icons/hicolor/scalable/apps/kmu-auv-simulator.svg
)
for path in "${required[@]}"; do
  [[ -s "$ROOT/$path" ]] || fail "required DEB path missing: $path"
done
pass "native DEB layout"

bash -n "$ROOT/usr/bin/kmu-auv-simulator" \
  "$ROOT/usr/bin/kmu-auv-simulator-installer" \
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
  auv_buoy_vision_control auv_lane_vision_control auv_web_gui
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
[[ -s "$TMP_DIR/ros/auv_lane_vision_control/src/surface_buoy_mission_node.cpp" ]] || \
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
pass "competition installer contracts"

echo "[verify-release] package OK: $DEB_PATH"
