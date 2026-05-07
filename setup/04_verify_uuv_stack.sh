#!/usr/bin/env bash
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SETUP_DIR}/.." && pwd)"
HELPERS_SH="${SETUP_DIR}/_setup_path_helpers.sh"
if [[ -f "$HELPERS_SH" ]]; then
  # shellcheck source=/dev/null
  source "$HELPERS_SH"
fi
if ! declare -F resolve_ros_workspace_dir >/dev/null 2>&1; then
  resolve_ros_workspace_dir() {
    local workspace_dir="$1"
    if [[ -n "${ROS_WORKSPACE_DIR:-}" ]]; then
      printf '%s\n' "${ROS_WORKSPACE_DIR}"
      return 0
    fi
    if [[ -d "${workspace_dir}/rospkg" ]]; then
      printf '%s\n' "${workspace_dir}/rospkg"
      return 0
    fi
    printf '%s\n' "${workspace_dir}"
  }
fi
if ! declare -F resolve_kmu26_auv_dir >/dev/null 2>&1; then
  resolve_kmu26_auv_dir() {
    local workspace_dir="$1"
    local ros_workspace_dir="$2"
    if [[ -n "${KMU26_AUV_DIR:-}" ]]; then
      printf '%s\n' "${KMU26_AUV_DIR}"
      return 0
    fi
    if [[ -d "${ros_workspace_dir}/kmu26_auv" ]]; then
      printf '%s\n' "${ros_workspace_dir}/kmu26_auv"
      return 0
    fi
    if [[ -d "${workspace_dir}/kmu26_auv" ]]; then
      printf '%s\n' "${workspace_dir}/kmu26_auv"
      return 0
    fi
    printf '%s\n' "${ros_workspace_dir}/kmu26_auv"
  }
fi
if ! declare -F preferred_setup_script_names >/dev/null 2>&1; then
  preferred_setup_script_names() {
    local user_shell
    user_shell="$(basename "${SHELL:-}")"
    if [[ "$user_shell" == "zsh" ]]; then
      printf '%s\n' \
        setup.zsh local_setup.zsh \
        setup.bash local_setup.bash \
        setup.sh local_setup.sh
      return 0
    fi

    printf '%s\n' \
      setup.bash local_setup.bash \
      setup.zsh local_setup.zsh \
      setup.sh local_setup.sh
  }
fi
if ! declare -F resolve_setup_script_in_dir >/dev/null 2>&1; then
  resolve_setup_script_in_dir() {
    local dir="$1"
    local script_name
    [[ -d "$dir" ]] || return 1

    while IFS= read -r script_name; do
      if [[ -f "${dir}/${script_name}" ]]; then
        printf '%s\n' "${dir}/${script_name}"
        return 0
      fi
    done < <(preferred_setup_script_names)

    return 1
  }
fi
if ! declare -F resolve_ros_install_setup >/dev/null 2>&1; then
  resolve_ros_install_setup() {
    local workspace_dir="$1"
    local ros_workspace_dir="$2"
    local setup_script
    if [[ -n "${ROS_INSTALL_SETUP:-}" && -f "${ROS_INSTALL_SETUP}" ]]; then
      printf '%s\n' "${ROS_INSTALL_SETUP}"
      return 0
    fi
    if setup_script="$(resolve_setup_script_in_dir "${ros_workspace_dir}/install")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
    if setup_script="$(resolve_setup_script_in_dir "${workspace_dir}/install")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
    printf '%s\n' "${ros_workspace_dir}/install/setup.bash"
  }
fi
if ! declare -F resolve_ros_env_setup >/dev/null 2>&1; then
  resolve_ros_env_setup() {
    local ros_distro="${1:-${ROS_DISTRO:-humble}}"
    local setup_script

    if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
      printf '%s\n' "${ROS_ENV_SETUP}"
      return 0
    fi

    for base_dir in \
      "/opt/ros/${ros_distro}" \
      "/usr/local/ros/${ros_distro}" \
      "/opt/homebrew/opt/ros/${ros_distro}" \
      "/usr/local/opt/ros/${ros_distro}"
    do
      if setup_script="$(resolve_setup_script_in_dir "${base_dir}")"; then
        printf '%s\n' "${setup_script}"
        return 0
      fi
    done

    if [[ -n "${CONDA_PREFIX:-}" ]]; then
      if setup_script="$(resolve_setup_script_in_dir "${CONDA_PREFIX}")"; then
        printf '%s\n' "${setup_script}"
        return 0
      fi
    fi

    return 1
  }
fi
if ! declare -F resolve_qgc_app >/dev/null 2>&1; then
  resolve_qgc_app() {
    local os_name
    local workspace_dir

    if [[ -n "${QGC_APP:-}" ]]; then
      printf '%s\n' "${QGC_APP}"
      return 0
    fi

    workspace_dir="${WORKSPACE_DIR}"

    for candidate in \
      "${workspace_dir}/QGroundControl.AppImage" \
      "${workspace_dir}/QGroundControl-x86_64.AppImage" \
      "$HOME/Applications/QGroundControl.AppImage" \
      "${workspace_dir}/QGroundControl.app" \
      "/Applications/QGroundControl.app" \
      "$HOME/Applications/QGroundControl.app"
    do
      if [[ -e "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
    done

    if command -v QGroundControl >/dev/null 2>&1; then
      command -v QGroundControl
      return 0
    fi
    if command -v qgroundcontrol >/dev/null 2>&1; then
      command -v qgroundcontrol
      return 0
    fi

    os_name="$(uname -s)"
    if [[ "$os_name" == "Darwin" ]]; then
      printf '%s\n' "/Applications/QGroundControl.app"
    else
      printf '%s\n' "${workspace_dir}/QGroundControl.AppImage"
    fi
  }
fi
ENV_FILE="${WORKSPACE_DIR}/.uuv_mujoco_env.sh"
if [[ -f "$ENV_FILE" ]]; then
  # shellcheck source=/dev/null
  source "$ENV_FILE"
fi

resolve_default_mj311_root() {
  if [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/uuv_mujoco"
    return 0
  fi
  if [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/mujoco311"
    return 0
  fi
  printf '%s\n' "$HOME/.venvs/uuv_mujoco"
}

UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${WORKSPACE_DIR}/uuv_mujoco}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-$(resolve_ros_workspace_dir "${WORKSPACE_DIR}")}"
KMU26_AUV_DIR="${KMU26_AUV_DIR:-$(resolve_kmu26_auv_dir "${WORKSPACE_DIR}" "${ROS_WORKSPACE_DIR}")}"
ROS_INSTALL_SETUP="${ROS_INSTALL_SETUP:-$(resolve_ros_install_setup "${WORKSPACE_DIR}" "${ROS_WORKSPACE_DIR}")}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_ENV_SETUP="${ROS_ENV_SETUP:-$(resolve_ros_env_setup "${ROS_DISTRO}" 2>/dev/null || true)}"
MJ311_ROOT="${MJ311_ROOT:-$(resolve_default_mj311_root)}"
QGC_APP="${QGC_APP:-$(resolve_qgc_app)}"
BUILD_REAL_PKG=0

usage() {
  cat <<'USAGE'
Usage: ./setup/04_verify_uuv_stack.sh [options]

Run this after 03_setup_uuv_mujoco.sh.

What it checks:
  - workspace layout
  - ardupilot checkout and sim_vehicle.py
  - uuv_mujoco extraction and key launch scripts
  - virtualenv/python dependencies
  - QGroundControl presence
  - optional ROS2/MAVROS availability
  - optional kmu26_auv build

Options:
  --build-real-pkg      If ROS2 and kmu26_auv are present, run colcon build
  -h, --help            Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-real-pkg)
      BUILD_REAL_PKG=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[error] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  echo "[PASS] $1"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  echo "[WARN] $1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  echo "[FAIL] $1"
}

has_cmd() {
  command -v "$1" >/dev/null 2>&1
}

run_clean_env() {
  env -u PYTHONPATH -u PYTHONHOME "$@"
}

ros_activation_hint() {
  if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
    printf '%s\n' "source \"${ROS_ENV_SETUP}\""
    return 0
  fi
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '%s\n' "source /opt/ros/${ROS_DISTRO}/setup.bash"
    return 0
  fi
  printf '%s\n' "# activate your ROS2 environment first"
}

check_path() {
  local path="$1"
  local label="$2"
  if [[ -e "$path" ]]; then
    pass "${label}: ${path}"
  else
    fail "${label}: missing (${path})"
  fi
}

echo "[verify] workspace: ${WORKSPACE_DIR}"
echo "[verify] ros workspace: ${ROS_WORKSPACE_DIR}"
echo "[verify] uuv_mujoco: ${UUV_MUJOCO_DIR}"
echo "[verify] ardupilot: ${ARDUPILOT_DIR}"
echo "[verify] kmu26_auv: ${KMU26_AUV_DIR}"
echo "[verify] venv: ${MJ311_ROOT}"
echo "[verify] qgc: ${QGC_APP}"
echo "[verify] ros env setup: ${ROS_ENV_SETUP:-<none>}"
echo "[verify] ros install setup: ${ROS_INSTALL_SETUP}"

check_path "${WORKSPACE_DIR}" "workspace directory"
check_path "${ROS_WORKSPACE_DIR}" "ros workspace directory"
check_path "${ARDUPILOT_DIR}" "ardupilot directory"
check_path "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" "sim_vehicle.py"
check_path "${UUV_MUJOCO_DIR}" "uuv_mujoco directory"
check_path "${UUV_MUJOCO_DIR}/v2.2" "uuv_mujoco/v2.2"
check_path "${UUV_MUJOCO_DIR}/uuv_control_gui.py" "uuv_control_gui.py"
check_path "${UUV_MUJOCO_DIR}/v2.2/start_sitl_mujoco_mj311.sh" "start_sitl_mujoco_mj311.sh"
check_path "${UUV_MUJOCO_DIR}/v2.2/start_ardusub_sitl_mj311.sh" "start_ardusub_sitl_mj311.sh"
check_path "${UUV_MUJOCO_DIR}/v2.2/launch_uuv_sim.sh" "launch_uuv_sim.sh"
check_path "${UUV_MUJOCO_DIR}/v2.2/reset_uuv_sim.sh" "reset_uuv_sim.sh"
check_path "${UUV_MUJOCO_DIR}/v2.2/scenes/tank_legacy_scene.xml" "scenes/tank_legacy_scene.xml"
check_path "${UUV_MUJOCO_DIR}/v2.2/scenes/tank_current_scene.xml" "scenes/tank_current_scene.xml"
check_path "${UUV_MUJOCO_DIR}/v2.2/config/sim_profiles.json" "config/sim_profiles.json"
check_path "${UUV_MUJOCO_DIR}/v2.2/config/thruster_params.json" "config/thruster_params.json"
check_path "${UUV_MUJOCO_DIR}/v2.2/config/thruster_performance.json" "config/thruster_performance.json"
check_path "${MJ311_ROOT}/bin/python" "venv python"

for script in \
  "${UUV_MUJOCO_DIR}/v2.2/start_sitl_mujoco_mj311.sh" \
  "${UUV_MUJOCO_DIR}/v2.2/start_ardusub_sitl_mj311.sh" \
  "${UUV_MUJOCO_DIR}/v2.2/launch_uuv_sim.sh" \
  "${UUV_MUJOCO_DIR}/v2.2/reset_uuv_sim.sh"
do
  if [[ -f "$script" ]]; then
    if bash -n "$script"; then
      pass "shell syntax ok: $script"
    else
      fail "shell syntax error: $script"
    fi
  fi
done

if [[ -x "${MJ311_ROOT}/bin/python" ]]; then
  if run_clean_env "${MJ311_ROOT}/bin/python" - <<'PY'
import mujoco
import numpy
import pymavlink
import MAVProxy
import pexpect
import PIL
import future
import dronecan
import em
print("python dependency check ok")
print("MuJoCo Version:", mujoco.__version__)
PY
  then
    pass "python dependencies import successfully"
  else
    fail "python dependency import failed"
  fi
fi

if [[ -f "$QGC_APP" || -d "$QGC_APP" ]]; then
  pass "QGroundControl path exists"
else
  warn "QGroundControl path missing: ${QGC_APP}"
fi

if has_cmd ros2; then
  pass "ROS2 command available in current environment: $(command -v ros2)"
  if ros2 pkg prefix mavros >/dev/null 2>&1; then
    pass "ROS2 package available: mavros"
  else
    warn "ROS2 package missing: mavros"
  fi
  if ros2 pkg prefix mavros_msgs >/dev/null 2>&1; then
    pass "ROS2 package available: mavros_msgs"
  else
    warn "ROS2 package missing: mavros_msgs"
  fi
  if ros2 pkg prefix rviz2 >/dev/null 2>&1; then
    pass "ROS2 package available: rviz2"
  else
    warn "ROS2 package missing: rviz2"
  fi
  if [[ -x "${MJ311_ROOT}/bin/python" ]]; then
    if "${MJ311_ROOT}/bin/python" - <<'PY'
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
print('ros2 python bridge imports ok')
PY
    then
      pass "ROS2 Python bridge imports successfully in the MuJoCo venv"
    else
      warn "ROS2 Python bridge import failed in ${MJ311_ROOT}; rerun 03_setup_uuv_mujoco.sh with a ROS2-compatible Python interpreter"
    fi
  fi
elif [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  pass "ROS2 setup exists: /opt/ros/${ROS_DISTRO}/setup.bash"
  if bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 pkg prefix mavros >/dev/null 2>&1"; then
    pass "ROS2 package available: mavros"
  else
    warn "ROS2 package missing: mavros"
  fi
  if bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 pkg prefix mavros_msgs >/dev/null 2>&1"; then
    pass "ROS2 package available: mavros_msgs"
  else
    warn "ROS2 package missing: mavros_msgs"
  fi
  if bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 pkg prefix rviz2 >/dev/null 2>&1"; then
    pass "ROS2 package available: rviz2"
  else
    warn "ROS2 package missing: rviz2"
  fi
  if [[ -x "${MJ311_ROOT}/bin/python" ]]; then
    if bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && '${MJ311_ROOT}/bin/python' - <<'PY'
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
print('ros2 python bridge imports ok')
PY"; then
      pass "ROS2 Python bridge imports successfully in the MuJoCo venv"
    else
      warn "ROS2 Python bridge import failed in ${MJ311_ROOT}; rerun 03_setup_uuv_mujoco.sh with a ROS2-compatible Python interpreter"
    fi
  fi
else
  warn "ROS2 is not active in PATH and no /opt/ros/${ROS_DISTRO}/setup.bash was found"
fi

if [[ -d "$KMU26_AUV_DIR" ]]; then
  pass "kmu26_auv source directory exists"
  check_path "${KMU26_AUV_DIR}/package.xml" "kmu26_auv package.xml"
  if [[ "$BUILD_REAL_PKG" -eq 1 ]]; then
    if has_cmd colcon; then
      if has_cmd ros2; then
        if (
          cd "${ROS_WORKSPACE_DIR}" &&
          colcon build --base-paths "${KMU26_AUV_DIR}"
        ); then
          pass "kmu26_auv colcon build succeeded"
        else
          fail "kmu26_auv colcon build failed"
        fi
      elif [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
        if bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && cd '${ROS_WORKSPACE_DIR}' && colcon build --base-paths '${KMU26_AUV_DIR}'"; then
          pass "kmu26_auv colcon build succeeded"
        else
          fail "kmu26_auv colcon build failed"
        fi
      else
        fail "cannot build kmu26_auv because ROS2 is not active"
      fi
    else
      fail "colcon not found"
    fi
  else
    warn "kmu26_auv exists but build was not requested"
  fi
else
  warn "kmu26_auv source directory missing: ${KMU26_AUV_DIR}"
fi

echo
echo "[verify] summary"
echo "[verify] PASS=${PASS_COUNT} WARN=${WARN_COUNT} FAIL=${FAIL_COUNT}"
echo

echo "[verify] recommended run commands"
if has_cmd ros2 && [[ -f "${ROS_INSTALL_SETUP}" ]]; then
  echo
  cat <<EOF
# terminal 1 (default: MuJoCo + SITL + ROS2 bridge, QGC video off)
cd "${UUV_MUJOCO_DIR}/v2.2"
./reset_uuv_sim.sh --with-qgc-stop
./start_sitl_mujoco_mj311.sh --with-qgc-stop -- --fluid-model custom

# optional external MAVROS / kmu26_auv stack
# terminal 1
cd "${UUV_MUJOCO_DIR}/v2.2"
./reset_uuv_sim.sh --with-qgc-stop
./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat -- --fluid-model ellipsoid

# terminal 2
$(ros_activation_hint)
source "${ROS_INSTALL_SETUP}"
ros2 launch hit25_auv_ros2 rov_start.launch.py fcu_url:=udp://:14551@127.0.0.1:14551

# terminal 3
$(ros_activation_hint)
rviz2 -d "${KMU26_AUV_DIR}/rviz/rov.rviz"
EOF
elif has_cmd ros2 || [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo
  cat <<EOF
# default: MuJoCo + SITL + ROS2 bridge, QGC video off
cd "${UUV_MUJOCO_DIR}/v2.2"
./reset_uuv_sim.sh --with-qgc-stop
./start_sitl_mujoco_mj311.sh --with-qgc-stop -- --fluid-model custom

# optional no-ROS mode
cd "${UUV_MUJOCO_DIR}/v2.2"
./start_sitl_mujoco_mj311.sh --no-ros2 -- --fluid-model ellipsoid
EOF
else
  echo
  cat <<EOF
cd "${UUV_MUJOCO_DIR}/v2.2"
./reset_uuv_sim.sh --with-qgc-stop
./start_sitl_mujoco_mj311.sh --with-qgc-stop -- --fluid-model custom
EOF
fi

if (( FAIL_COUNT > 0 )); then
  exit 1
fi
