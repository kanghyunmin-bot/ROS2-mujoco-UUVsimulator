#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PACKAGE_NAME="uuv_sim_current_ubuntu22.04"
ARCHIVE_PATH="${1:-${ROOT_DIR}/documentary/release/builds/latest/${PACKAGE_NAME}.zip}"

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[PASS] $*"
}

for cmd in unzip zipinfo python3 sha256sum; do
  command -v "$cmd" >/dev/null 2>&1 || fail "command missing: ${cmd}"
done

[[ -f "$ARCHIVE_PATH" ]] || fail "archive missing: $ARCHIVE_PATH"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

unzip -q "$ARCHIVE_PATH" -d "$TMP_DIR/direct"
ROOT="${TMP_DIR}/direct/${PACKAGE_NAME}"
[[ -d "$ROOT" ]] || fail "package root missing after unzip: ${PACKAGE_NAME}"

required=(
  README_FIRST.md PORTABILITY_CHECKLIST.md BUNDLE_CONTENTS.txt RELEASE_MANIFEST.txt VERSION
  install_uuv_sim_current_ubuntu22.sh preflight_uuv_sim_current.sh verify_current_dist.sh
  install_and_run_web.sh run_control_gui.sh uuv_control_gui.py cleanup_generated_artifacts.sh
  setup_kmu26_hydrophone.sh
  uuv_mujoco.zip sim/current/assets/yolo/best.pt rospkg/README.md
  rospkg/dvl_msgs.zip rospkg/ping360_sonar_msgs.zip
  rospkg/kmu26_auv_msg.zip rospkg/kmu26_auv.zip
  rospkg/audio_common_msgs.zip rospkg/audio_common.zip rospkg/audio_capture.zip
  rospkg/kmu26_auv_buoy_vision_control.zip rospkg/kmu26_mission_fsm.zip
  rospkg/kmu26_auv_web_gui.zip rospkg/kmu26_pinger_homing.zip
  rospkg/robot_localization.zip
)
for path in "${required[@]}"; do
  [[ -s "${ROOT}/${path}" ]] || fail "missing or empty package path: ${path}"
done
pass "top-level distribution payload is complete"

grep -Fq '`kmu26_pinger_homing`' "${ROOT}/rospkg/README.md" \
  || fail "ROS source ownership README is missing standalone pinger documentation"
pass "ROS source ownership README is packaged"

bash -n \
  "${ROOT}/install_uuv_sim_current_ubuntu22.sh" \
  "${ROOT}/preflight_uuv_sim_current.sh" \
  "${ROOT}/verify_current_dist.sh" \
  "${ROOT}/install_and_run_web.sh" \
  "${ROOT}/run_control_gui.sh" \
  "${ROOT}/cleanup_generated_artifacts.sh" \
  "${ROOT}/setup_kmu26_hydrophone.sh"
pass "packaged shell entry points parse"

for marker in \
  'mujoco==3.8.0' 'opencv-python-headless<5' 'ultralytics' \
  'download.pytorch.org/whl/cpu' 'mavros-extras' 'rqt-image-view' \
  'kmu26_auv_buoy_vision_control.zip' 'audio_capture.zip' 'audio_common.zip' \
  'kmu26_mission_fsm.zip' 'kmu26_auv_web_gui.zip' 'kmu26_pinger_homing.zip' \
  'robot_localization.zip' 'ROS_SOURCE_DIR' \
  'UUV_YOLO_MODEL' '--run-web' '--run-headless' '.uuv_runtime_payload_version' \
  'UUV_SIM_FORCE_REFRESH' \
  '.upgrade_tmp.$$' 'user course, sonar, physics settings, logs, and generated state were preserved' \
  'removing superseded vision FSM from previous release' \
  'workspace version committed' 'ROS sources refreshed; skipping ROS helper package build'
do
  grep -Fq -- "$marker" "${ROOT}/install_uuv_sim_current_ubuntu22.sh" \
    || fail "installer missing required behavior marker: ${marker}"
done
pass "installer contains current runtime, model and ROS setup contracts"

bad_pattern='(^|/)(\.git|__pycache__|build|install|log|logs|generated|private)(/|$)|\.py[co]$|\.(bag|db3|mcap)$|(^|/)best \(1\)\.pt$|(^|/)MUJOCO_LOG\.TXT$|(^|/)core(\.[0-9]+)?$|(~$|\.orig$|\.bak($|_))'
if zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E "$bad_pattern" >/dev/null; then
  zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E "$bad_pattern" | sed -n '1,40p'
  fail "runtime archive contains generated, build, VCS or log artifacts"
fi
for archive in "${ROOT}"/rospkg/*.zip; do
  if zipinfo -1 "$archive" | grep -E "$bad_pattern" >/dev/null; then
    zipinfo -1 "$archive" | grep -E "$bad_pattern" | sed -n '1,40p'
    fail "ROS source archive contains generated/build/VCS artifacts: $(basename "$archive")"
  fi
done
if zipinfo -1 "$ARCHIVE_PATH" | grep -E '(^|/)ardupilot/|QGroundControl.*AppImage|real_robot_ros_bag' >/dev/null; then
  fail "distribution unexpectedly embeds ArduPilot, QGC or rosbag data"
fi
pass "archives exclude machine-specific and generated data"

mkdir -p "${TMP_DIR}/runtime" "${TMP_DIR}/runtime/rospkg"
unzip -q "${ROOT}/uuv_mujoco.zip" -d "${TMP_DIR}/runtime"
for archive in "${ROOT}"/rospkg/*.zip; do
  unzip -q "$archive" -d "${TMP_DIR}/runtime/rospkg"
done
cp -a "${ROOT}/run_control_gui.sh" "${TMP_DIR}/runtime/run_control_gui.sh"
cp -a "${ROOT}/uuv_control_gui.py" "${TMP_DIR}/runtime/uuv_control_gui.py"

CURRENT="${TMP_DIR}/runtime/sim/current"
ROS_ROOT="${TMP_DIR}/runtime/rospkg"
nested=(
  "${CURRENT}/run_uuv_mujoco.py"
  "${CURRENT}/launch_uuv_sim.sh"
  "${CURRENT}/start_sitl_mujoco_mj311.sh"
  "${CURRENT}/gui/web_control_gui.py"
  "${CURRENT}/gui/yolo_buoy_detector.py"
  "${CURRENT}/gui/sim_stack_env_defaults.py"
  "${CURRENT}/gui/sim_stack_launch_command.py"
  "${CURRENT}/sim/runtime/model_runtime_setup.py"
  "${CURRENT}/tools/check_external_fsm_mavros_contract.py"
  "${CURRENT}/scenes/tank_current_scene.xml"
  "${ROS_ROOT}/dvl_msgs/package.xml"
  "${ROS_ROOT}/ping360_sonar_msgs/package.xml"
  "${ROS_ROOT}/kmu26_auv_msg/package.xml"
  "${ROS_ROOT}/kmu26_auv/package.xml"
  "${ROS_ROOT}/kmu26_auv/src/joy2mavros.cpp"
  "${ROS_ROOT}/kmu26_auv/test/test_joy2mavros_mux_input.py"
  "${ROS_ROOT}/audio_common_msgs/package.xml"
  "${ROS_ROOT}/audio_common/package.xml"
  "${ROS_ROOT}/audio_capture/package.xml"
  "${ROS_ROOT}/audio_capture/src/audio_phase_estimator.cpp"
  "${ROS_ROOT}/audio_capture/src/snr_gradient_homing_node.cpp"
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/package.xml"
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/models/best.pt"
  "${ROS_ROOT}/kmu26_mission_fsm/package.xml"
  "${ROS_ROOT}/kmu26_auv_web_gui/package.xml"
  "${ROS_ROOT}/kmu26_auv_web_gui/kmu26_auv_web_gui/server.py"
  "${ROS_ROOT}/kmu26_auv_web_gui/web/index.html"
  "${ROS_ROOT}/kmu26_pinger_homing/package.xml"
  "${ROS_ROOT}/kmu26_pinger_homing/src/pinger_homing/pinger_homing_controller.cpp"
  "${ROS_ROOT}/kmu26_pinger_homing/src/rc_override_mux.cpp"
  "${ROS_ROOT}/kmu26_pinger_homing/launch/pinger_homing_real.launch.py"
  "${ROS_ROOT}/robot_localization/package.xml"
)
for path in "${nested[@]}"; do
  [[ -s "$path" ]] || fail "nested required file missing or empty: $path"
done
pass "runtime and ROS source archives extract with required files"

python3 - "$ROS_ROOT" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1])
expected = {
    "audio_capture",
    "audio_common",
    "audio_common_msgs",
    "auv_buoy_vision_control",
    "dvl_msgs",
    "hit25_auv_ros2",
    "hit25_auv_ros2_msg",
    "kmu26_auv_web_gui",
    "kmu26_mission_fsm",
    "kmu26_pinger_homing",
    "ping360_sonar_msgs",
    "robot_localization",
}
found = []
for package_xml in sorted(root.glob("*/package.xml")):
    name = ET.parse(package_xml).getroot().findtext("name")
    if name:
        found.append(name.strip())
if len(found) != len(set(found)):
    raise SystemExit(f"duplicate ROS package names in DIST: {found}")
if set(found) != expected:
    missing = sorted(expected - set(found))
    extra = sorted(set(found) - expected)
    raise SystemExit(f"ROS package set mismatch: missing={missing} extra={extra}")
print(f"active ROS package set ok ({len(found)} packages)")
PY
pass "exact 12-package ROS source contract passes"

gui_model_sha="$(sha256sum "${ROOT}/sim/current/assets/yolo/best.pt" | awk '{print $1}')"
ros_model_sha="$(sha256sum "${ROS_ROOT}/kmu26_auv_buoy_vision_control/models/best.pt" | awk '{print $1}')"
[[ "$gui_model_sha" == "$ros_model_sha" ]] || fail "GUI and ROS best.pt hashes differ"
[[ "$(stat -c %s "${ROOT}/sim/current/assets/yolo/best.pt")" -gt 1000000 ]] || fail "best.pt is unexpectedly small"
pass "best.pt is present and identical in GUI/ROS model locations (${gui_model_sha})"

grep -Fq '"UUV_MUJOCO_TIMESTEP": "0.008"' "${CURRENT}/gui/sim_stack_env_defaults.py" \
  || fail "stable 0.008 timestep default missing"
grep -Fq 'DEFAULT_CAMERA_PRESET_ID = "hd720_realtime"' "${CURRENT}/gui/sim_stack_launch_command.py" \
  || fail "1280x720@30Hz camera preset is not the default"
grep -Fq '"UUV_COURSE_BUOY_TRACK_CSV_ENABLE": "0"' "${CURRENT}/gui/sim_stack_env_defaults.py" \
  || fail "high-volume buoy CSV logging is enabled by default"
pass "current performance and camera defaults are packaged"

python3 -m py_compile \
  "${ROOT}/uuv_control_gui.py" \
  "${CURRENT}/run_uuv_mujoco.py" \
  "${CURRENT}/gui/"*.py \
  "${CURRENT}/sim/runtime/model_runtime_setup.py" \
  "${CURRENT}/tools/check_external_fsm_mavros_contract.py" \
  "${CURRENT}/tools/check_homing_direction_viewer.py" \
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/scripts/buoy_vision_core.py" \
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/scripts/yolo_buoy_detector.py" \
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/test/test_launch_contract.py" \
  "${ROS_ROOT}/kmu26_auv/test/test_joy2mavros_mux_input.py" \
  "${ROS_ROOT}/kmu26_auv_web_gui/kmu26_auv_web_gui/server.py" \
  "${ROS_ROOT}/kmu26_auv_web_gui/kmu26_auv_web_gui/ros_interface.py" \
  "${ROS_ROOT}/kmu26_auv_web_gui/launch/gui_server.launch.py" \
  "${ROS_ROOT}/kmu26_auv_web_gui/test/test_ros_topic_contract.py" \
  "${ROS_ROOT}/kmu26_pinger_homing/launch/pinger_homing_real.launch.py" \
  "${ROS_ROOT}/kmu26_pinger_homing/launch/pinger_homing_gui.launch.py"
pass "packaged Python entry points compile"

grep -Fq '"require_source_lock", legacy_python_sequence_' \
  "${ROS_ROOT}/kmu26_pinger_homing/src/pinger_homing/pinger_homing_controller.cpp" \
  || fail "standalone pinger controller lost its validated Phase/SNR source-lock split"
grep -Fq 'package="kmu26_pinger_homing"' \
  "${ROS_ROOT}/kmu26_pinger_homing/launch/pinger_homing_real.launch.py" \
  || fail "standalone pinger launch does not own its C++ controller"
grep -Fq 'default="kmu26_pinger_homing"' \
  "${ROS_ROOT}/kmu26_auv_web_gui/kmu26_auv_web_gui/server.py" \
  || fail "web GUI does not default to the standalone pinger package"
python3 "${ROS_ROOT}/kmu26_pinger_homing/test/test_standalone_package_contract.py"
pass "standalone C++ pinger and web GUI ownership contracts pass"

python3 "${CURRENT}/tools/check_gui_start_contract.py"
python3 "${CURRENT}/tools/check_web_gui_contract.py"
python3 "${CURRENT}/tools/check_sim_runtime_smooth_contract.py"
python3 "${CURRENT}/tools/check_competition_course_scene.py"
python3 "${CURRENT}/tools/check_hydrophone_audio_timing.py"
python3 "${CURRENT}/tools/check_homing_direction_viewer.py"
python3 "${ROS_ROOT}/kmu26_auv_buoy_vision_control/test/test_buoy_vision_core.py"
python3 "${ROS_ROOT}/kmu26_auv_buoy_vision_control/test/test_launch_contract.py"
python3 "${ROS_ROOT}/kmu26_auv_web_gui/test/test_ros_topic_contract.py"
grep -Fq '"rc_output_topic": joy_rc_output_topic' \
  "${ROS_ROOT}/kmu26_auv/launch/rov_start.launch.py" \
  || fail "real vehicle joystick cannot be routed through the exclusive RC mux"
grep -Fq '"joystick_topic": "/control/joystick/rc_override"' \
  "${ROS_ROOT}/kmu26_auv_buoy_vision_control/launch/buoy_vision_mission.launch.py" \
  || fail "vision mux does not accept physical joystick override"
pass "packaged runtime and vision contract checks pass"

echo "[verify-current-dist] package ok: ${ARCHIVE_PATH}"
