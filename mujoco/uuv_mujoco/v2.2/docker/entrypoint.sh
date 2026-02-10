#!/usr/bin/env bash
set -e

# Source ROS2 when available so bridge mode works without extra shell setup.
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

exec "$@"
