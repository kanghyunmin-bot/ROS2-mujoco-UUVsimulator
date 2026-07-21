#!/usr/bin/env bash

set -euo pipefail

topic="${1:-/dvl/config/command}"

publish_command() {
  local command="$1"
  local parameter_name="${2:-}"
  local parameter_value="${3:-}"

  echo "[dvl_setup] ${command} ${parameter_name} ${parameter_value}"
  ros2 topic pub --once "${topic}" dvl_msgs/msg/ConfigCommand \
    "{command: ${command}, parameter_name: '${parameter_name}', parameter_value: '${parameter_value}'}"
}

publish_command get_config
sleep 0.5
publish_command set_config acoustic_enabled true
sleep 0.5
publish_command set_config range_mode auto
sleep 0.5
# publish_command calibrate_gyro
# sleep 0.5
publish_command reset_dead_reckoning

echo "[dvl_setup] completed on ${topic}"
