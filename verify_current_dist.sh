#!/usr/bin/env bash
set -euo pipefail

DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${DIST_DIR}/out"
PACKAGE_NAME="uuv_sim_current_ubuntu22.04"
ARCHIVE_PATH="${1:-${OUT_DIR}/${PACKAGE_NAME}.zip}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[PASS] $*"
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "command missing: $1"
}

require_cmd unzip
require_cmd zipinfo
require_cmd python3

[[ -f "$ARCHIVE_PATH" ]] || fail "archive missing: $ARCHIVE_PATH"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

unzip -q "$ARCHIVE_PATH" -d "$TMP_DIR"
ROOT="${TMP_DIR}/${PACKAGE_NAME}"
[[ -d "$ROOT" ]] || fail "package root missing after unzip"

for path in \
  README_FIRST.md \
  PORTABILITY_CHECKLIST.md \
  BUNDLE_CONTENTS.txt \
  RELEASE_MANIFEST.txt \
  VERSION \
  install_uuv_sim_current_ubuntu22.sh \
  preflight_uuv_sim_current.sh \
  verify_current_dist.sh \
  install_and_run_web.sh \
  run_control_gui.sh \
  uuv_control_gui.py \
  cleanup_generated_artifacts.sh \
  uuv_mujoco.zip \
  rospkg/kmu26_auv.zip \
  rospkg/dvl_msgs.zip \
  rospkg/ping360_sonar_msgs.zip
do
  [[ -e "${ROOT}/${path}" ]] || fail "missing package path: ${path}"
done
pass "top-level package paths exist"

bash -n \
  "${ROOT}/install_uuv_sim_current_ubuntu22.sh" \
  "${ROOT}/preflight_uuv_sim_current.sh" \
  "${ROOT}/verify_current_dist.sh" \
  "${ROOT}/install_and_run_web.sh" \
  "${ROOT}/run_control_gui.sh" \
  "${ROOT}/cleanup_generated_artifacts.sh"
pass "shell syntax ok"

for marker in \
  "xwayland" \
  "libdecor-0-0" \
  "libosmesa6" \
  "libglfw3" \
  "QT_QPA_PLATFORM" \
  "mujoco.viewer" \
  "mavros-extras" \
  "rqt-image-view" \
  "--run-headless" \
  "--run-web" \
  "--skip-python-env" \
  "preflight_uuv_sim_current.sh" \
  ".uuv_sim_current_version" \
  ".uuv_runtime_payload_version"
do
  grep -Fq -- "$marker" "${ROOT}/install_uuv_sim_current_ubuntu22.sh" "${ROOT}/preflight_uuv_sim_current.sh" \
    || fail "installer/preflight missing marker: ${marker}"
done
pass "installer/preflight include display, ROS, Python and run-mode checks"

if zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E '(^|/)(__pycache__|logs|\.git|private)(/|$)|\.py[co]$|(^|/)MUJOCO_LOG\.TXT$|(\.bak($|_)|~$|\.orig$)' >/dev/null; then
  zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E '(^|/)(__pycache__|logs|\.git|private)(/|$)|\.py[co]$|(^|/)MUJOCO_LOG\.TXT$|(\.bak($|_)|~$|\.orig$)' | sed -n '1,40p'
  fail "uuv_mujoco.zip contains generated/private/cache/log files"
fi
pass "uuv_mujoco.zip excludes generated/private/cache/log files"

if zipinfo -1 "$ARCHIVE_PATH" | grep -E '(^|/)ardupilot/|QGroundControl.*AppImage|real_robot_ros_bag' >/dev/null; then
  zipinfo -1 "$ARCHIVE_PATH" | grep -E '(^|/)ardupilot/|QGroundControl.*AppImage|real_robot_ros_bag' | sed -n '1,40p'
  fail "archive includes excluded large external assets"
fi
pass "archive excludes ArduPilot, QGC binary, and rosbag data"

unzip -q "${ROOT}/uuv_mujoco.zip" -d "${TMP_DIR}/runtime"
unzip -q "${ROOT}/rospkg/kmu26_auv.zip" -d "${TMP_DIR}/rospkg"
unzip -q "${ROOT}/rospkg/dvl_msgs.zip" -d "${TMP_DIR}/rospkg"
unzip -q "${ROOT}/rospkg/ping360_sonar_msgs.zip" -d "${TMP_DIR}/rospkg"

CURRENT="${TMP_DIR}/runtime/uuv_mujoco/current"
for path in \
  "${CURRENT}/run_uuv_mujoco.py" \
  "${CURRENT}/launch_uuv_sim.sh" \
  "${CURRENT}/start_sitl_mujoco_mj311.sh" \
  "${CURRENT}/gui/web_control_gui.py" \
  "${CURRENT}/gui/sim_stack_env_defaults.py" \
  "${CURRENT}/sim/runtime/model_runtime_setup.py" \
  "${CURRENT}/tools/check_buoy_collector_capture.py" \
  "${CURRENT}/tools/check_gui_start_contract.py" \
  "${CURRENT}/tools/check_web_gui_contract.py" \
  "${CURRENT}/tools/check_model_runtime_setup.py" \
  "${CURRENT}/scenes/tank_current_scene.xml" \
  "${TMP_DIR}/rospkg/kmu26_auv/package.xml" \
  "${TMP_DIR}/rospkg/dvl_msgs/package.xml" \
  "${TMP_DIR}/rospkg/ping360_sonar_msgs/package.xml"
do
  [[ -e "$path" ]] || fail "nested required path missing: $path"
done
pass "nested current runtime and ROS package paths exist"

grep -Fq '"UUV_MUJOCO_TIMESTEP": "0.005"' "${CURRENT}/gui/sim_stack_env_defaults.py" \
  || fail "current GUI low profile timestep is not 0.005"
grep -Fq '"UUV_COURSE_BUOY_TRACK_CSV_ENABLE": "0"' "${CURRENT}/gui/sim_stack_env_defaults.py" \
  || fail "course buoy tracking is not disabled by default"
grep -Fq 'course buoy contact timestep guard' "${CURRENT}/sim/runtime/model_runtime_setup.py" \
  || fail "current runtime lacks course buoy timestep guard"
grep -Fq 'check_gui_low_profile_red_buoy_contact_stability' "${CURRENT}/tools/check_buoy_collector_capture.py" \
  || fail "current buoy contact stability test missing"
if grep -Eq 'viewer_surface_disc|surface_projection' "${CURRENT}/scenes/tank_current_scene.xml"; then
  fail "removed buoy projection/contact-disc visual helpers reappeared in current scene"
fi
pass "current buoy-contact stability changes are packaged"

python3 -m py_compile \
  "${ROOT}/uuv_control_gui.py" \
  "${CURRENT}/run_uuv_mujoco.py" \
  "${CURRENT}/gui/"*.py \
  "${CURRENT}/sim/runtime/model_runtime_setup.py" \
  "${CURRENT}/sim/runtime/course_buoy_runtime.py" \
  "${CURRENT}/tools/check_buoy_collector_capture.py" \
  "${CURRENT}/tools/check_gui_start_contract.py" \
  "${CURRENT}/tools/check_web_gui_contract.py" \
  "${CURRENT}/tools/check_model_runtime_setup.py" \
  "${CURRENT}/tools/check_competition_course_scene.py"
pass "python syntax ok for packaged current runtime"

python3 "${CURRENT}/tools/check_gui_start_contract.py"
UUV_WEB_GUI_WORKSPACE="${ROOT}" python3 "${CURRENT}/tools/check_web_gui_contract.py"
python3 "${CURRENT}/tools/check_model_runtime_setup.py"
python3 "${CURRENT}/tools/check_competition_course_scene.py"
pass "packaged current contract checks pass"

echo "[verify-current-dist] package ok: ${ARCHIVE_PATH}"
