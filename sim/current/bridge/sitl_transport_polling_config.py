"""MAVLink and JSON polling-budget configuration for SitlTransport."""

from __future__ import annotations

import numpy as np

from bridge.sitl_env import env_to_float


def initialize_transport_polling_state(transport: object) -> None:
    mavlink_poll_default_hz = (
        50.0
        if transport._sitl_json_servo_fallback
        else max(50.0, 3.0 * transport._sitl_mavlink_servo_hz)
    )
    transport._sitl_mavlink_poll_hz = float(
        np.clip(env_to_float("ROS2_UUV_SITL_MAVLINK_POLL_HZ", mavlink_poll_default_hz), 5.0, 200.0)
    )
    transport._sitl_command_poll_hz = float(
        np.clip(env_to_float("ROS2_UUV_SITL_COMMAND_POLL_HZ", 400.0), 20.0, 400.0)
    )
    transport._sitl_mavlink_poll_budget = int(
        np.clip(round(env_to_float("ROS2_UUV_SITL_MAVLINK_POLL_BUDGET", 48.0)), 1, 256)
    )
    transport._sitl_command_poll_budget = int(
        np.clip(round(env_to_float("ROS2_UUV_SITL_COMMAND_POLL_BUDGET", 24.0)), 1, 128)
    )
    transport._sitl_json_poll_budget = int(
        np.clip(round(env_to_float("ROS2_UUV_SITL_JSON_POLL_BUDGET", 96.0)), 1, 512)
    )
    transport._sitl_next_mavlink_poll_wall = 0.0
    transport._sitl_next_command_poll_wall = 0.0


__all__ = ["initialize_transport_polling_state"]
