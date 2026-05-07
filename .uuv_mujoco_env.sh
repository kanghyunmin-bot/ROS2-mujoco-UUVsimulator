#!/usr/bin/env bash

if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  _UUV_ENV_SOURCE="${BASH_SOURCE[0]}"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  _UUV_ENV_SOURCE="${(%):-%N}"
else
  _UUV_ENV_SOURCE="$0"
fi

export WORKSPACE_DIR="$(cd "$(dirname "${_UUV_ENV_SOURCE}")" && pwd)"
export ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-${WORKSPACE_DIR}/rospkg}"
export UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${WORKSPACE_DIR}/uuv_mujoco}"
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
export KMU26_AUV_DIR="${KMU26_AUV_DIR:-${ROS_WORKSPACE_DIR}/kmu26_auv}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ -z "${ROS_INSTALL_SETUP:-}" ]]; then
  for _candidate in \
    "${ROS_WORKSPACE_DIR}/install/setup.bash" \
    "${ROS_WORKSPACE_DIR}/install/setup.zsh" \
    "${WORKSPACE_DIR}/install/setup.bash" \
    "${WORKSPACE_DIR}/install/setup.zsh"
  do
    if [[ -f "${_candidate}" ]]; then
      export ROS_INSTALL_SETUP="${_candidate}"
      break
    fi
  done
fi

if [[ -z "${ROS_ENV_SETUP:-}" ]]; then
  for _candidate in \
    "/opt/ros/${ROS_DISTRO}/setup.bash" \
    "/opt/ros/${ROS_DISTRO}/setup.zsh" \
    "$HOME/miniconda3/envs/ros2_h311/setup.bash" \
    "$HOME/miniconda3/envs/ros2_h311/setup.zsh" \
    "$HOME/miniconda3/envs/ros2/setup.bash" \
    "$HOME/miniconda3/envs/ros2/setup.zsh"
  do
    if [[ -f "${_candidate}" ]]; then
      export ROS_ENV_SETUP="${_candidate}"
      break
    fi
  done
fi

if [[ -z "${MJ311_ROOT:-}" ]]; then
  if [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    export MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
  elif [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    export MJ311_ROOT="$HOME/.venvs/mujoco311"
  else
    export MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
  fi
fi

export MJ311_PYTHON="${MJ311_PYTHON:-${MJ311_ROOT}/bin/python}"
export MJ311_MJPYTHON="${MJ311_MJPYTHON:-${MJ311_ROOT}/bin/mjpython}"

if [[ -z "${QGC_APP:-}" ]]; then
  for _candidate in \
    "${WORKSPACE_DIR}/dist/ubuntu22.04/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/dist/ubuntu22.04/QGroundControl-x86_64.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl-x86_64.AppImage" \
    "$HOME/Downloads/QGroundControl.AppImage" \
    "$HOME/Downloads/QGroundControl-x86_64.AppImage" \
    "$HOME/Applications/QGroundControl.AppImage" \
    "$HOME/bin/QGroundControl.AppImage" \
    "/opt/QGroundControl/QGroundControl.AppImage" \
    "/usr/local/bin/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl.app" \
    "/Applications/QGroundControl.app" \
    "$HOME/Applications/QGroundControl.app"
  do
    if [[ -e "${_candidate}" ]]; then
      export QGC_APP="${_candidate}"
      break
    fi
  done
fi
