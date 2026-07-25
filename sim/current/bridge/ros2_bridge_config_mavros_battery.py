"""MAVROS battery default configuration."""

from __future__ import annotations

from bridge.sitl_env import env_to_float


def configure_mavros_battery_defaults(bridge: object) -> None:
    bridge._mavros_battery_voltage = float(env_to_float("ROS2_UUV_MAVROS_BATTERY_VOLTAGE", 16.0))
    bridge._mavros_battery_current = float(env_to_float("ROS2_UUV_MAVROS_BATTERY_CURRENT", 0.0))
    bridge._mavros_battery_soc = float(env_to_float("ROS2_UUV_MAVROS_BATTERY_SOC", 100.0))


__all__ = ["configure_mavros_battery_defaults"]
