#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export UUV_SITL_BACKEND_DEFAULT="${UUV_SITL_BACKEND_DEFAULT:-native}"
export UUV_SITL_BACKEND="${UUV_SITL_BACKEND:-${UUV_SITL_BACKEND_DEFAULT}}"
if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/lib/librmw_fastrtps_cpp.so" ]]; then
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  elif [[ -f "/opt/ros/${ROS_DISTRO:-humble}/lib/librmw_cyclonedds_cpp.so" ]]; then
    export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  else
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
  fi
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DISABLE_DAEMON="${ROS_DISABLE_DAEMON:-1}"
if [[ -z "${PYTHON_BIN:-}" && -x "${HOME}/miniconda3/envs/ros2_h311/bin/python" ]]; then
  export PYTHON_BIN="${HOME}/miniconda3/envs/ros2_h311/bin/python"
fi
exec "${ROOT_DIR}/run_control_gui.sh" "$@"
