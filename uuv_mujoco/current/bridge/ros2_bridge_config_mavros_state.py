"""MAVROS state defaults for the ROS2 bridge."""

from __future__ import annotations

from bridge.sitl_env import env_to_float


def configure_mavros_state_defaults(bridge: object) -> None:
    bridge._mavros_mode = "MANUAL"
    bridge._mavros_armed = False
    bridge._mavros_state_pub_hz = float(env_to_float("ROS2_UUV_MAVROS_STATE_HZ", 20.0))
    bridge._mavros_state_next_t = 0.0
    bridge._ros_sensor_rate_next_t: dict[str, float] = {}


__all__ = ["configure_mavros_state_defaults"]
