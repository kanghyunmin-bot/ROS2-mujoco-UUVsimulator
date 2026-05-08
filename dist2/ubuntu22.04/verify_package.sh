#!/usr/bin/env bash
set -euo pipefail

DIST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${DIST_DIR}/out"
PACKAGE_NAME="uuv_sim_ubuntu22.04_dist2"
ARCHIVE_PATH="${1:-${OUT_DIR}/${PACKAGE_NAME}.zip}"

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[verify-dist2] command not found: $1" >&2
    exit 1
  }
}

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[PASS] $*"
}

require_cmd unzip
require_cmd zipinfo
require_cmd python3

[[ -f "${ARCHIVE_PATH}" ]] || fail "archive missing: ${ARCHIVE_PATH}"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

unzip -q "${ARCHIVE_PATH}" -d "${TMP_DIR}"
ROOT="${TMP_DIR}/${PACKAGE_NAME}"

for path in \
  "README_FIRST.md" \
  "README.md" \
  "DIST_GUIDE.md" \
  "PORTABILITY_AUDIT.md" \
  "BUNDLE_CONTENTS.txt" \
  "RELEASE_MANIFEST.txt" \
  "install_uuv_sim_ubuntu22.sh" \
  "install_and_run.sh" \
  "run_control_gui.sh" \
  "run_control_gui_ubuntu.sh" \
  "uuv_control_gui.py" \
  "cleanup_generated_artifacts.sh" \
  "uuv_mujoco.zip" \
  "rospkg/kmu26_auv.zip" \
  "rospkg/dvl_msgs.zip" \
  "rospkg/ping360_sonar_msgs.zip" \
  "document/docsource/run_uuv_param_autotune.py" \
  "document/docsource/replay_april1_real_commands_in_mujoco.py" \
  "document/docsource/analyze_april1_real_bags.py" \
  "document/docsource/run_closed_loop_april1_replay.sh" \
  "document/docsource/replay_april1_rc_override_closed_loop.py" \
  "document/docsource/compare_closed_loop_april1_replay.py"
do
  [[ -e "${ROOT}/${path}" ]] || fail "missing package path: ${path}"
done
pass "top-level package paths exist"

bash -n \
  "${ROOT}/install_uuv_sim_ubuntu22.sh" \
  "${ROOT}/install_and_run.sh" \
  "${ROOT}/run_control_gui.sh" \
  "${ROOT}/run_control_gui_ubuntu.sh" \
  "${ROOT}/cleanup_generated_artifacts.sh" \
  "${ROOT}/document/docsource/run_closed_loop_april1_replay.sh"
pass "top-level shell syntax ok"

for marker in \
  "--native-python" \
  "--python-mode" \
  "--mujoco-pip-spec" \
  "--run-after-install" \
  "--run-headless" \
  'PYTHON_MODE="${PYTHON_MODE:-native}"' \
  "cleanup_duplicate_ros_apt_repo" \
  "setuptools<80" \
  "python3-tk" \
  "python3-dev" \
  "libosmesa6" \
  "can-utils" \
  "ros-\${ROS_DISTRO}-std-srvs" \
  "ros-\${ROS_DISTRO}-tf2-msgs" \
  "ros-\${ROS_DISTRO}-mavros-extras" \
  "ros-\${ROS_DISTRO}-rqt-image-view" \
  "python-pptx" \
  "rosbags" \
  "dvl_msgs" \
  "ping360_sonar_msgs"
do
  if ! grep -Fq -- "$marker" "${ROOT}/install_uuv_sim_ubuntu22.sh"; then
    fail "installer missing expected dependency/option marker: ${marker}"
  fi
done
pass "installer includes current dependency and run-option markers"

if ! grep -Fq "Source commit:" "${ROOT}/RELEASE_MANIFEST.txt"; then
  fail "release manifest missing source commit"
fi
pass "release manifest records source metadata"

if zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E '(^|/)(__pycache__|logs|\.git)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' >/dev/null; then
  zipinfo -1 "${ROOT}/uuv_mujoco.zip" | grep -E '(^|/)(__pycache__|logs|\.git)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' | sed -n '1,40p'
  fail "uuv_mujoco.zip contains generated/cache/backup files"
fi
pass "uuv_mujoco.zip excludes generated/cache/backup files"

if zipinfo -1 "${ROOT}/rospkg/kmu26_auv.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' >/dev/null; then
  zipinfo -1 "${ROOT}/rospkg/kmu26_auv.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' | sed -n '1,40p'
  fail "kmu26_auv.zip contains git/generated/cache/backup files"
fi
pass "kmu26_auv.zip excludes git/generated/cache/backup files"

if zipinfo -1 "${ROOT}/rospkg/dvl_msgs.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' >/dev/null; then
  zipinfo -1 "${ROOT}/rospkg/dvl_msgs.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' | sed -n '1,40p'
  fail "dvl_msgs.zip contains git/generated/cache/backup files"
fi
pass "dvl_msgs.zip excludes git/generated/cache/backup files"

if zipinfo -1 "${ROOT}/rospkg/ping360_sonar_msgs.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' >/dev/null; then
  zipinfo -1 "${ROOT}/rospkg/ping360_sonar_msgs.zip" | grep -E '(^|/)(\.git|__pycache__)(/|$)|\.py[co]$|(^|/)\.DS_Store$|(\.bak($|_)|~$|\.orig$)' | sed -n '1,40p'
  fail "ping360_sonar_msgs.zip contains git/generated/cache/backup files"
fi
pass "ping360_sonar_msgs.zip excludes git/generated/cache/backup files"

if zipinfo -1 "${ARCHIVE_PATH}" | grep -E '(^|/)real_robot_ros_bag/|(^|/)QGroundControl\.app/|(^|/)ardupilot/' >/dev/null; then
  zipinfo -1 "${ARCHIVE_PATH}" | grep -E '(^|/)real_robot_ros_bag/|(^|/)QGroundControl\.app/|(^|/)ardupilot/' | sed -n '1,40p'
  fail "dist2 archive contains excluded large runtime sources"
fi
pass "dist2 archive excludes large external assets"

unzip -q "${ROOT}/uuv_mujoco.zip" -d "${TMP_DIR}/runtime"
unzip -q "${ROOT}/rospkg/kmu26_auv.zip" -d "${TMP_DIR}/rospkg"
unzip -q "${ROOT}/rospkg/dvl_msgs.zip" -d "${TMP_DIR}/rospkg"
unzip -q "${ROOT}/rospkg/ping360_sonar_msgs.zip" -d "${TMP_DIR}/rospkg"

if grep -RIlE --exclude='DIST_GUIDE.md' --exclude='PORTABILITY_AUDIT.md' '/Users/kanghyunmin|PYTHONNOUSERSITE=1' "${ROOT}" "${TMP_DIR}/runtime" "${TMP_DIR}/rospkg" | sed -n '1,40p' | grep -q .; then
  grep -RIlE --exclude='DIST_GUIDE.md' --exclude='PORTABILITY_AUDIT.md' '/Users/kanghyunmin|PYTHONNOUSERSITE=1' "${ROOT}" "${TMP_DIR}/runtime" "${TMP_DIR}/rospkg" | sed -n '1,40p'
  fail "package contains host-specific paths or user-site blocking env"
fi
pass "package excludes host-specific paths and user-site blocking env"

for path in \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/run_urdf_full.py" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/launch_uuv_sim.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/reset_uuv_sim.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/gui/uuv_control_gui.py" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/gui/app.py" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/config/sim_profiles.json" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/config/thruster_params.json" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/scenes/tank_current_scene.xml" \
  "${TMP_DIR}/rospkg/kmu26_auv/package.xml" \
  "${TMP_DIR}/rospkg/kmu26_auv/CMakeLists.txt" \
  "${TMP_DIR}/rospkg/dvl_msgs/package.xml" \
  "${TMP_DIR}/rospkg/dvl_msgs/CMakeLists.txt" \
  "${TMP_DIR}/rospkg/dvl_msgs/msg/DVL.msg" \
  "${TMP_DIR}/rospkg/dvl_msgs/msg/ConfigCommand.msg" \
  "${TMP_DIR}/rospkg/ping360_sonar_msgs/package.xml" \
  "${TMP_DIR}/rospkg/ping360_sonar_msgs/CMakeLists.txt" \
  "${TMP_DIR}/rospkg/ping360_sonar_msgs/msg/SonarEcho.msg"
do
  [[ -e "${path}" ]] || fail "nested required path missing: ${path}"
done
pass "nested runtime paths exist"

bash -n \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/launch_uuv_sim.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/reset_uuv_sim.sh"
pass "nested shell syntax ok"

python3 -m py_compile \
  "${ROOT}/uuv_control_gui.py" \
  "${ROOT}/document/docsource/"*.py \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/run_urdf_full.py" \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/bridge/"*.py \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/gui/"*.py \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/physics/"*.py \
  "${TMP_DIR}/runtime/uuv_mujoco/v2.2/tools/"*.py \
  "${TMP_DIR}/rospkg/kmu26_auv/launch/"*.py \
  "${TMP_DIR}/rospkg/kmu26_auv/scripts/"*.py
pass "python syntax ok"

echo "[verify-dist2] package ok: ${ARCHIVE_PATH}"
