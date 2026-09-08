#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

# Named volumes are created as root on first use. Hand only the two dedicated
# cache mounts to the unprivileged development user before building.
sudo mkdir -p /workspace/ardupilot/build /home/robot/.ccache
sudo chown -R robot:robot /workspace/ardupilot/build /home/robot/.ccache

if [[ -f /workspace/.uuv_mujoco_env.sh ]]; then
  source /workspace/.uuv_mujoco_env.sh
fi

if [[ -f /workspace/rospkg/install/setup.bash ]]; then
  source /workspace/rospkg/install/setup.bash
fi

git config --global --add safe.directory /workspace >/dev/null 2>&1 || true
git config --global --add safe.directory /workspace/ardupilot >/dev/null 2>&1 || true

exec "$@"
