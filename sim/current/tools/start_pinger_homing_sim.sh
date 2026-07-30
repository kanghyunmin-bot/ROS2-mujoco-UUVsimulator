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
# Text overlay updates call mujoco.viewer.set_texts() on the physics-loop
# thread. The native viewer lock can stall there long enough to reduce RTF to
# ~0.1 and make MAVROS time out the 1 Hz SITL heartbeat. Homing needs the
# viewer, not its diagnostic text decoration.
export UUV_MUJOCO_VIEWER_TEXT_OVERLAY="0"
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
export UUV_EKF_CONTRACT="${UUV_EKF_CONTRACT:-althold_baro}"
export ROS2_UUV_ASYNC_CAMERA_RENDER="1"
export ROS2_UUV_HYDROPHONE_AUDIO_HZ="23.4375"
export ROS2_UUV_HYDROPHONE_SYNC_HZ="50"
export ROS2_UUV_HYDROPHONE_STATUS_HZ="10"
export ROS2_UUV_HYDROPHONE_NOISE_PROFILE="${ROS2_UUV_HYDROPHONE_NOISE_PROFILE:-rosbag_20260707}"
# The competition pinger starts about 22 m from the vehicle.  At 0.020 the
# inverse-sqrt range model produced only 0.5--2.0 IQ SNR against the measured
# July 7 receiver floor, below the real estimator's 2.0 acceptance gate.  That
# left the Phase controller with zero delta-range samples.  Keep the measured
# noise profile unchanged and model a pinger with enough source level to be
# detectable at the configured start range.
export ROS2_UUV_HYDROPHONE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_AMPLITUDE:-0.040}"
export ROS2_UUV_HYDROPHONE_NOISE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_NOISE_AMPLITUDE:-0.005}"
export ROS2_UUV_HYDROPHONE_SNR_PROBE_NOISE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_SNR_PROBE_NOISE_AMPLITUDE:-0.0}"
export ROS2_UUV_HYDROPHONE_PHASE_NOISE_STD_RAD="${ROS2_UUV_HYDROPHONE_PHASE_NOISE_STD_RAD:-0.05}"
export ROS2_UUV_HYDROPHONE_PHASE_NOISE_CORRELATION_S="2.0"
export ROS2_UUV_HYDROPHONE_INTERFERERS_ENABLE="0"
export ROS2_UUV_HYDROPHONE_ROSBAG_ACTIVITY_REFERENCE="0.12"
export ROS2_UUV_HYDROPHONE_ROSBAG_BROADBAND_RMS="${ROS2_UUV_HYDROPHONE_ROSBAG_BROADBAND_RMS:-0.34}"
export ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_PROBABILITY="${ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_PROBABILITY:-0.07}"
export ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_AMPLITUDE:-1.50}"
export ROS2_UUV_HYDROPHONE_ROSBAG_COMMON_FRACTION="1.0"
export ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_HZ="21332.8645"
export ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_AMPLITUDE:-0.021}"

# Lightweight homing may disable camera/YOLO work, but it must not turn the
# canonical course scene into inert display geometry.  CourseBuoyRuntime is
# responsible for buoyancy, physical rake release, and collector capture, so
# it remains enabled in both profiles.
export UUV_COURSE_BUOYS_ENABLE="1"

echo "[pinger-sim] profile=${PROFILE} dt=${UUV_MUJOCO_TIMESTEP}s viewer=${UUV_MUJOCO_VIEWER_FPS}fps"
echo "[pinger-sim] PCM=96kHz/S32LE/2ch, noise=rosbag_20260707, course_buoys=${UUV_COURSE_BUOYS_ENABLE}"

exec "${SIM_DIR}/start_sitl_mujoco_mj311.sh" \
  --ros2-real-pkg-compat \
  -- \
  --scene "${SIM_DIR}/scenes/tank_current_scene.xml" \
  --fluid-model current \
  --initial-bar30-depth-m 8.30 \
  --ros2-sensor-hz 100 \
  --thruster-loop-hz 100 \
  --viewer-fps 12 \
  --profile current \
  "${EXTRA_ARGS[@]}"
