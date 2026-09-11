#!/usr/bin/env bash
# Apply the April 2 pressure observation profile to an explicit simulator command.
set -euo pipefail
if (( $# == 0 )); then
    echo 'Usage: run_bag0402_sensor_profile.sh <simulator-command> [arguments...]' >&2
    exit 2
fi
rov_profile_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH="$rov_profile_root/config/sensor_models/imu_bar30_bag_20260402.json"
# Existing per-sensor overrides take precedence in the runtime. Reject conflicts
# instead of silently running a different profile from the one being evaluated.
for rov_profile_key in $(compgen -e); do
    case "$rov_profile_key" in
        ROS2_UUV_IMU_SENSOR_*|ROS2_UUV_BAR30_SENSOR_*)
            echo "Unset conflicting override before this profile: $rov_profile_key" >&2
            exit 2
            ;;
    esac
done
exec "$@"
