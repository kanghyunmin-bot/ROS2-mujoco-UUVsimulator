"""Environment-derived sensor replay initialization config."""

from __future__ import annotations

import os

import numpy as np

from bridge.sitl_env import env_flag, env_to_float


def configure_sensor_replay_timing(transport: object) -> None:
    transport._sensor_replay_time_offset_s = env_to_float(
        "ROS2_UUV_SITL_SENSOR_REPLAY_TIME_OFFSET_S",
        0.0,
    )
    transport._sensor_replay_real_start_s = env_to_float(
        "ROS2_UUV_SITL_SENSOR_REPLAY_REAL_START_S",
        0.0,
    )
    transport._sensor_replay_start_on_rc = env_flag(
        "ROS2_UUV_SITL_SENSOR_REPLAY_START_ON_RC",
        False,
    )
    transport._sensor_replay_start_delay_s = max(
        0.0,
        env_to_float("ROS2_UUV_SITL_SENSOR_REPLAY_START_DELAY_S", 0.0),
    )


def configure_sensor_replay_reply_policy(transport: object) -> None:
    transport._sensor_replay_live_rangefinder = env_flag(
        "ROS2_UUV_SITL_SENSOR_REPLAY_LIVE_RANGEFINDER",
        False,
    )
    transport._sensor_replay_immediate_reply = env_flag(
        "ROS2_UUV_SITL_SENSOR_REPLAY_IMMEDIATE_REPLY",
        False,
    )
    transport._sensor_replay_immediate_last_frame_count: int | None = None
    transport._sensor_replay_immediate_send_counter = 0
    transport._sensor_replay_immediate_last_log_wall = -1.0


def configure_native_vpd_start_policy(transport: object) -> None:
    transport._native_vpd_cursor = 0
    transport._native_vpd_started = False
    transport._native_vpd_last_replay_t_s: float | None = None
    transport._native_vpd_last_log_wall = -1.0
    transport._native_vpd_start_tolerance_s = float(
        np.clip(
            env_to_float("ROS2_UUV_SITL_SENSOR_REPLAY_VPD_START_TOLERANCE_S", 0.2),
            0.0,
            2.0,
        )
    )


def configure_sensor_replay_clock(transport: object) -> None:
    transport._sensor_replay_clock = os.getenv(
        "ROS2_UUV_SITL_SENSOR_REPLAY_CLOCK",
        "sim_time",
    ).strip().lower()
    if transport._sensor_replay_clock in {"sim_time", "servo_frame"}:
        return
    print(
        "[sitl_transport] unknown ROS2_UUV_SITL_SENSOR_REPLAY_CLOCK="
        f"{transport._sensor_replay_clock!r}; falling back to sim_time",
        flush=True,
    )
    transport._sensor_replay_clock = "sim_time"


__all__ = [
    "configure_native_vpd_start_policy",
    "configure_sensor_replay_clock",
    "configure_sensor_replay_reply_policy",
    "configure_sensor_replay_timing",
]
