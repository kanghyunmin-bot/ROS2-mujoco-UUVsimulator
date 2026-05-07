#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ROOT_DIR}/.uuv_mujoco_env.sh"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  source "${ENV_FILE}"
else
  export WORKSPACE_DIR="${ROOT_DIR}"
  export UUV_MUJOCO_DIR="${ROOT_DIR}/uuv_mujoco"
  export ARDUPILOT_DIR="${ROOT_DIR}/ardupilot"
  export ROS_WORKSPACE_DIR="${ROOT_DIR}/rospkg"
  export ROS_DISTRO="${ROS_DISTRO:-humble}"
fi

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

if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
  source_ros_setup_safely "${ROS_ENV_SETUP}"
elif [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  source_ros_setup_safely "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

if [[ -n "${ROS_INSTALL_SETUP:-}" && -f "${ROS_INSTALL_SETUP}" ]]; then
  source_ros_setup_safely "${ROS_INSTALL_SETUP}"
elif [[ -f "${ROS_WORKSPACE_DIR:-${ROOT_DIR}/rospkg}/install/setup.bash" ]]; then
  source_ros_setup_safely "${ROS_WORKSPACE_DIR:-${ROOT_DIR}/rospkg}/install/setup.bash"
fi

export PYTHONNOUSERSITE=1
exec python3 "${ROOT_DIR}/uuv_control_gui.py" "$@"
