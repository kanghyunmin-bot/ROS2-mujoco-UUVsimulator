#!/usr/bin/env bash
set -euo pipefail

# Reproducible competition-tank launcher for phase/SNR homing.  The default
# lightweight profile removes only unrelated course-buoy runtime wrenches;
# vehicle, SITL, hydrophone PCM, water physics, pinger site and viewer remain
# live.  Use --full-physics for the three-minute full-scene acceptance pass.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROFILE="lightweight"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: start_pinger_homing_sim.sh [--lightweight|--full-physics] [-- <extra simulator args>]

  --lightweight  Pinger-only runtime load, viewer enabled (default; ~1 min gate)
  --full-physics Keep all 25 course-buoy wrenches (3 min/full-scene gate)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lightweight)
      PROFILE="lightweight"
      shift
      ;;
    --full-physics)
      PROFILE="full"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

export UUV_RUNTIME_PROFILE="balanced"
export UUV_MUJOCO_TIMESTEP="0.005"
export UUV_COURSE_BUOY_TIMESTEP_GUARD="1"
export UUV_COURSE_BUOY_TRACK_CSV_ENABLE="0"
export UUV_COURSE_BUOY_UPDATE_HZ="10"
export UUV_MUJOCO_VIEWER_FPS="12"
export UUV_MUJOCO_VIEWER_WIDTH="960"
export UUV_MUJOCO_VIEWER_HEIGHT="540"
export UUV_MUJOCO_SHADOW_SIZE="512"
export UUV_MUJOCO_OFFSAMPLES="1"
export UUV_MUJOCO_VIEWER_TEXT_OVERLAY="1"
# Low-cost rendering/audio settings must not lower the closed-loop attitude
# cadence: 30/40 Hz drives the real ArduSub ATC tuning into a limit cycle.
export UUV_ROS2_SENSOR_HZ="100"
export UUV_THRUSTER_LOOP_HZ="100"
export SITL_SENSOR_HZ_DEFAULT="100"
export SITL_THRUSTER_LOOP_HZ_DEFAULT="100"
export SITL_MAVLINK_SERVO_HZ_DEFAULT="30"
export SITL_SPEEDUP_DEFAULT="1"
# The raw-IMU-only ALT_HOLD profile leaves yaw unobservable (compass/GPS/yaw
# source are disabled), so gyro bias makes MAVROS/EKF yaw drift away from the
# MuJoCo vehicle.  The bridge already sends VISION_POSITION_DELTA.  Enable the
# existing ArduPilot MAV/VISO backend for this homing acceptance profile so the
# real-package localization stack receives a coherent attitude/velocity frame.
export UUV_EKF_CONTRACT="althold_baro"
export ROS2_UUV_ASYNC_CAMERA_RENDER="1"
export ROS2_UUV_HYDROPHONE_AUDIO_HZ="23.4375"
export ROS2_UUV_HYDROPHONE_SYNC_HZ="50"
export ROS2_UUV_HYDROPHONE_STATUS_HZ="10"

# Lightweight homing may disable camera/YOLO work, but it must not turn the
# canonical course scene into inert display geometry.  CourseBuoyRuntime is
# responsible for buoyancy, physical rake release, and collector capture, so
# it remains enabled in both profiles.
export UUV_COURSE_BUOYS_ENABLE="1"

echo "[pinger-sim] profile=${PROFILE} dt=${UUV_MUJOCO_TIMESTEP}s viewer=${UUV_MUJOCO_VIEWER_FPS}fps"
echo "[pinger-sim] PCM=96kHz/S32LE/2ch, normal pool+thruster noise, course_buoys=${UUV_COURSE_BUOYS_ENABLE}"

exec "${SIM_DIR}/start_sitl_mujoco_mj311.sh" \
  --ros2-real-pkg-compat \
  -- \
  --scene "${SIM_DIR}/scenes/tank_current_scene.xml" \
  --fluid-model current \
  --initial-bar30-depth-m auto \
  --ros2-sensor-hz 100 \
  --thruster-loop-hz 100 \
  --viewer-fps 12 \
  --profile current \
  "${EXTRA_ARGS[@]}"
