#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export UUV_SITL_BACKEND_DEFAULT="${UUV_SITL_BACKEND_DEFAULT:-native}"
export UUV_SITL_BACKEND="${UUV_SITL_BACKEND:-${UUV_SITL_BACKEND_DEFAULT}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DISABLE_DAEMON="${ROS_DISABLE_DAEMON:-1}"
if [[ -z "${PYTHON_BIN:-}" && -x "${HOME}/miniconda3/envs/ros2_h311/bin/python" ]]; then
  export PYTHON_BIN="${HOME}/miniconda3/envs/ros2_h311/bin/python"
fi
exec "${ROOT_DIR}/run_control_gui.sh" "$@"
