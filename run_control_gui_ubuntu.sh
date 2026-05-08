#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export WORKSPACE_DIR="${WORKSPACE_DIR:-${ROOT_DIR}}"
export UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${WORKSPACE_DIR}/uuv_mujoco}"
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
export ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-${WORKSPACE_DIR}/rospkg}"
export KMU26_AUV_DIR="${KMU26_AUV_DIR:-${ROS_WORKSPACE_DIR}/kmu26_auv}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"

ROS_ENV_SETUP="${ROS_ENV_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
ROS_INSTALL_SETUP="${ROS_INSTALL_SETUP:-${ROS_WORKSPACE_DIR}/install/setup.bash}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
GUI_ENTRY="${UUV_MUJOCO_DIR}/v2.2/gui/uuv_control_gui.py"

source_ros_setup_safely() {
  local setup_file="$1"
  local restore_nounset=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  # shellcheck source=/dev/null
  source "${setup_file}"
  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

if [[ ! -f "${ROS_ENV_SETUP}" ]]; then
  echo "[gui-ubuntu] ROS setup not found: ${ROS_ENV_SETUP}" >&2
  echo "[gui-ubuntu] Install ROS 2 Humble or set ROS_ENV_SETUP=/path/to/setup.bash." >&2
  exit 1
fi
source_ros_setup_safely "${ROS_ENV_SETUP}"

if [[ -f "${ROS_INSTALL_SETUP}" ]]; then
  source_ros_setup_safely "${ROS_INSTALL_SETUP}"
else
  echo "[gui-ubuntu] workspace ROS install not found yet: ${ROS_INSTALL_SETUP}" >&2
  echo "[gui-ubuntu] Continuing with system ROS only; use the GUI Build pkg button or run colcon build when package topics are needed." >&2
fi

if [[ ! -f "${GUI_ENTRY}" ]]; then
  echo "[gui-ubuntu] GUI entry not found: ${GUI_ENTRY}" >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${GUI_ENTRY}" "$@"
