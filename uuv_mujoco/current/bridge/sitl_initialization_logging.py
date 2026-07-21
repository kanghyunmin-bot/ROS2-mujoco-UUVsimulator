"""Sensor replay initialization logging helpers."""

from __future__ import annotations

import os


def log_native_vpd_replay_if_active(transport: object) -> None:
    if not transport._native_vpd_events:
        return
    first = transport._native_vpd_events[0]
    last = transport._native_vpd_events[-1]
    print(
        "[sitl_transport] native VISION_POSITION_DELTA replay active: "
        f"{len(transport._native_vpd_events)} events from "
        f"{os.getenv('ROS2_UUV_SITL_SENSOR_REPLAY_VPD_CSV')} "
        f"(replay_t={first.t_replay_s:.3f}..{last.t_replay_s:.3f}s, "
        "synthetic VPD disabled)",
        flush=True,
    )


def log_sensor_replay_if_active(transport: object) -> None:
    if not transport._sensor_replay_frames:
        return
    print(
        "[sitl_transport] controller-parity sensor replay active: "
        f"{len(transport._sensor_replay_frames)} frames from "
        f"{os.getenv('ROS2_UUV_SITL_SENSOR_REPLAY_PREVIEW_CSV')} "
        f"(time_offset={transport._sensor_replay_time_offset_s:.3f}s, "
        f"start_on_rc={int(transport._sensor_replay_start_on_rc)}, "
        f"start_delay={transport._sensor_replay_start_delay_s:.3f}s, "
        f"clock={transport._sensor_replay_clock}, "
        f"live_rangefinder={int(transport._sensor_replay_live_rangefinder)}, "
        f"immediate_reply={int(transport._sensor_replay_immediate_reply)})",
        flush=True,
    )


def enforce_immediate_reply_clock_contract(transport: object) -> None:
    if not transport._sensor_replay_frames:
        return
    if not transport._sensor_replay_immediate_reply:
        return
    if transport._sensor_replay_clock == "servo_frame":
        return
    print(
        "[sitl_transport] warning: immediate sensor replay reply requires "
        "ROS2_UUV_SITL_SENSOR_REPLAY_CLOCK=servo_frame; disabling immediate reply",
        flush=True,
    )
    transport._sensor_replay_immediate_reply = False


__all__ = [
    "enforce_immediate_reply_clock_contract",
    "log_native_vpd_replay_if_active",
    "log_sensor_replay_if_active",
]
