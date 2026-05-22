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

ROS_BASE_SETUP=""
for candidate in \
  "${ROS_ENV_SETUP:-}" \
  "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" \
  "${HOME}/miniconda3/envs/ros2_mavros/setup.bash" \
  "${HOME}/miniconda3/envs/ros2_h311/setup.bash" \
  "${HOME}/miniconda3/envs/ros2/setup.bash" \
  "${CONDA_PREFIX:-}/setup.bash"
do
  [[ -n "${candidate}" ]] || continue
  if [[ -f "${candidate}" ]]; then
    ROS_BASE_SETUP="${candidate}"
    break
  fi
done

if [[ -n "${ROS_BASE_SETUP}" ]]; then
  source_ros_setup_safely "${ROS_BASE_SETUP}"
else
  echo "[gui] warning: ROS setup not found; relying on current shell environment" >&2
fi

if [[ -n "${ROS_INSTALL_SETUP:-}" && -f "${ROS_INSTALL_SETUP}" ]]; then
  source_ros_setup_safely "${ROS_INSTALL_SETUP}"
elif [[ -f "${ROS_WORKSPACE_DIR:-${ROOT_DIR}/rospkg}/install/setup.bash" ]]; then
  source_ros_setup_safely "${ROS_WORKSPACE_DIR:-${ROOT_DIR}/rospkg}/install/setup.bash"
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
GUI_ENTRY="${UUV_MUJOCO_DIR:-${ROOT_DIR}/uuv_mujoco}/v2.2/gui/uuv_control_gui.py"

if [[ ! -f "${GUI_ENTRY}" ]]; then
  echo "[gui] GUI entry not found: ${GUI_ENTRY}" >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${GUI_ENTRY}" "$@"
