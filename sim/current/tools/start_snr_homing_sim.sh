#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# RC circle tracking still needs a stable ExternalNav-backed local position,
# while horizontal motion itself is sent only through RC override in ALT_HOLD.
export UUV_EKF_CONTRACT="poshold_extnav"
export SITL_EKF3_EXTNAV=1
export SITL_EKF3_EXTNAV_POSZ=1
export SITL_EKF3_EXTNAV_VELZ=6
# At the competition-map start the pinger is roughly 22 m away.  A 1.3 m
# circle changes 1/r by only about 1 dB, smaller than the frame-to-frame
# thruster-noise scatter.  Use 1/r^2 spreading and the corresponding stronger
# source level so the *spatial* SNR change remains observable without exposing
# the simulator's oracle direction.
export ROS2_UUV_HYDROPHONE_ATTENUATION_EXPONENT="${ROS2_UUV_HYDROPHONE_ATTENUATION_EXPONENT:-2.0}"
export ROS2_UUV_HYDROPHONE_AMPLITUDE="${ROS2_UUV_HYDROPHONE_AMPLITUDE:-0.90}"
# Deterministic competition-map controller test: no receiver, thruster,
# impulse, stationary-tone, interferer, or oscillator-phase noise.
export ROS2_UUV_HYDROPHONE_NOISE_PROFILE="none"
export ROS2_UUV_HYDROPHONE_NOISE_AMPLITUDE="0.0"
export ROS2_UUV_HYDROPHONE_SNR_PROBE_NOISE_AMPLITUDE="0.0"
export ROS2_UUV_HYDROPHONE_INTERFERERS_ENABLE="0"
export ROS2_UUV_HYDROPHONE_INTERFERER_COUNT="0"
export ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_AMPLITUDE="0.0"
export ROS2_UUV_HYDROPHONE_ROSBAG_BROADBAND_RMS="0.0"
export ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_PROBABILITY="0.0"
export ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_AMPLITUDE="0.0"
export ROS2_UUV_HYDROPHONE_PHASE_NOISE_STD_RAD="0.0"

exec "${SCRIPT_DIR}/start_pinger_homing_sim.sh" "$@"
