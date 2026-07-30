"""Validated simulator profile used when Pinger Start owns a stopped sim.

The normal GUI ``Start Sim`` path intentionally keeps its camera-oriented
defaults.  Pinger homing lowers viewer/camera/auxiliary load so the 96 kHz
audio producer, odometry and ``/clock`` stay synchronized, while retaining the
validated 100 Hz attitude feedback and plant-update cadence.
"""

from __future__ import annotations


PINGER_HOMING_SIM_PURPOSE = "pinger_homing"


def normalize_sim_start_purpose(purpose: str | None) -> str:
    value = str(purpose or "default").strip().lower()
    if value in {"", "default", "sim"}:
        return "default"
    if value == PINGER_HOMING_SIM_PURPOSE:
        return value
    raise ValueError(f"unsupported simulator start purpose: {purpose}")


def pinger_sim_environment() -> dict[str, str]:
    """Mirror ``tools/start_pinger_homing_sim.sh --lightweight``."""

    return {
        "UUV_RUNTIME_PROFILE": "balanced",
        "UUV_MUJOCO_TIMESTEP": "0.005",
        "UUV_COURSE_BUOY_TIMESTEP_GUARD": "1",
        "UUV_COURSE_BUOY_TRACK_CSV_ENABLE": "0",
        "UUV_COURSE_BUOY_UPDATE_HZ": "10",
        # Pinger Start is still the canonical competition scene.  Disabling
        # this runtime leaves visible buoy bodies with no buoyancy, rake
        # release, or collector-net state machine and makes the viewer physics
        # fundamentally different from a normal simulation run.  Keep the
        # cheap camera/YOLO paths disabled instead; buoy physics stays live.
        "UUV_COURSE_BUOYS_ENABLE": "1",
        "UUV_MUJOCO_VIEWER_FPS": "12",
        "UUV_MUJOCO_VIEWER_WIDTH": "960",
        "UUV_MUJOCO_VIEWER_HEIGHT": "540",
        "UUV_MUJOCO_SHADOW_SIZE": "512",
        "UUV_MUJOCO_OFFSAMPLES": "1",
        # mujoco.viewer.set_texts() shares the native viewer lock. On the
        # desktop X11 runtime that lock can stall for several seconds after a
        # long viewer session, coupling physics time to the overlay refresh
        # and stretching the 1 Hz SITL heartbeat past MAVROS' 10 s timeout.
        # Homing does not consume viewer text, so keep the visual window but
        # disable only this lock-heavy decoration.
        "UUV_MUJOCO_VIEWER_TEXT_OVERLAY": "0",
        # Do not trade attitude-loop stability for audio/rendering headroom.
        # 100/100 Hz is the validated STABILIZE/ALT_HOLD safety floor.
        "UUV_ROS2_SENSOR_HZ": "100",
        "UUV_THRUSTER_LOOP_HZ": "100",
        "SITL_SENSOR_HZ_DEFAULT": "100",
        "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
        "SITL_MAVLINK_SERVO_HZ_DEFAULT": "30",
        "SITL_SPEEDUP_DEFAULT": "1",
        # Homing must exercise the same observable sensor contract as the
        # vehicle: IMU + DVL + pressure.  ExternalNav is generated from the
        # MuJoCo pose and is therefore oracle data, not a controller input.
        "UUV_EKF_CONTRACT": "althold_baro",
        "ROS2_UUV_ASYNC_CAMERA_RENDER": "1",
        # Real-package compatibility enables ROS image rendering by default in
        # launch_uuv_sim.sh.  Homing consumes only PCM/odom/depth, so disable it
        # explicitly to preserve the audio/clock real-time budget.
        "UUV_REAL_PKG_CAMERA_ENABLE": "0",
        "ROS2_UUV_HYDROPHONE_AUDIO_HZ": "23.4375",
        "ROS2_UUV_HYDROPHONE_SYNC_HZ": "50",
        "ROS2_UUV_HYDROPHONE_STATUS_HZ": "10",
        # July 7 real-bag calibration.  The quiet opening provides the
        # receiver/phase baseline; the armed ALT_HOLD interval provides the
        # 23.8--24.4 kHz propulsion envelope and clipping distribution.
        "ROS2_UUV_HYDROPHONE_NOISE_PROFILE": "rosbag_20260707",
        # At the competition start (~22 m), 0.020 falls below the upstream
        # estimator's fixed 2.0 IQ-SNR gate.  This source level preserves the
        # measured receiver/noise profile while making the intended pinger
        # observable to the unmodified physical estimator.
        "ROS2_UUV_HYDROPHONE_AMPLITUDE": "0.040",
        "ROS2_UUV_HYDROPHONE_NOISE_AMPLITUDE": "0.005",
        "ROS2_UUV_HYDROPHONE_SNR_PROBE_NOISE_AMPLITUDE": "0.0",
        # The rosbag broadband/impulse floor remains active below.  Keep only
        # a small additional synthetic phase jitter for the short interactive
        # ABBA gate; 0.19 rad made consecutive bearings mutually orthogonal.
        "ROS2_UUV_HYDROPHONE_PHASE_NOISE_STD_RAD": "0.05",
        "ROS2_UUV_HYDROPHONE_PHASE_NOISE_CORRELATION_S": "2.0",
        # The calibrated profile replaces the old arbitrary near-frequency
        # tones with actuator-gated colored and impulsive noise.
        "ROS2_UUV_HYDROPHONE_INTERFERERS_ENABLE": "0",
        "ROS2_UUV_HYDROPHONE_ROSBAG_ACTIVITY_REFERENCE": "0.12",
        "ROS2_UUV_HYDROPHONE_ROSBAG_BROADBAND_RMS": "0.34",
        "ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_PROBABILITY": "0.07",
        "ROS2_UUV_HYDROPHONE_ROSBAG_IMPULSE_AMPLITUDE": "1.50",
        "ROS2_UUV_HYDROPHONE_ROSBAG_COMMON_FRACTION": "1.0",
        "ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_HZ": "21332.8645",
        "ROS2_UUV_HYDROPHONE_ROSBAG_STATIONARY_TONE_AMPLITUDE": "0.021",
    }


def pinger_sim_launch_args() -> list[str]:
    return [
        "--fluid-model",
        "current",
        "--initial-bar30-depth-m",
        # no_odom_phase is intentionally horizontal-only.  Start the
        # pinger-purpose runtime at the competition acoustic-source depth so
        # its calibrated 3-D range can actually reach the success radius.
        "8.30",
        # Interactive homing starts on the competition approach lane instead
        # of replaying the full 21 m transit on every button press.  The
        # pinger, vehicle dynamics, PCM and controller remain live.
        "--initial-position-xy",
        "-2.5",
        "-10.0",
        "--initial-rpy-rad",
        "0.0",
        "0.0",
        "0.0",
        "--ros2-sensor-hz",
        "100",
        "--thruster-loop-hz",
        "100",
        "--viewer-fps",
        "12",
        "--profile",
        "current",
    ]


__all__ = [
    "PINGER_HOMING_SIM_PURPOSE",
    "normalize_sim_start_purpose",
    "pinger_sim_environment",
    "pinger_sim_launch_args",
]
