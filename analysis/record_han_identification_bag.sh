#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_BAG_DIRECTORY" >&2
  exit 2
fi

output_bag=$1
if [[ -e "$output_bag" ]]; then
  echo "output already exists: $output_bag" >&2
  exit 2
fi

# Read-only recorder for HAN-first plant identification.  It does not publish
# RC override, arm, change mode, or otherwise take control of the vehicle.
exec ros2 bag record --storage sqlite3 --output "$output_bag" \
  /mavros/rc/out \
  /mavros/rc/in \
  /mavros/rc/override \
  /mavros/state \
  /mavros/imu/data \
  /dvl/data \
  /dvl/twist \
  /depth/pose \
  /odometry/filtered \
  /localization/path
