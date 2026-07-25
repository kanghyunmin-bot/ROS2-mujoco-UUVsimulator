"""Compatibility exports for MAVROS-compatible bridge configuration."""

from __future__ import annotations

from bridge.ros2_bridge_config_mavros_battery import configure_mavros_battery_defaults
from bridge.ros2_bridge_config_mavros_rcout import configure_mavros_rcout_policy
from bridge.ros2_bridge_config_mavros_replay import configure_mavros_replay_state
from bridge.ros2_bridge_config_mavros_sensor_rates import configure_ros_sensor_rates, log_ros_sensor_rates
from bridge.ros2_bridge_config_mavros_state import configure_mavros_state_defaults


def configure_mavros_state_and_rates(bridge: object) -> None:
    configure_mavros_state_defaults(bridge)
    configure_ros_sensor_rates(bridge)
    configure_mavros_rcout_policy(bridge)
    log_ros_sensor_rates(bridge)
    configure_mavros_battery_defaults(bridge)
    configure_mavros_replay_state(bridge)


__all__ = [
    "configure_mavros_state_defaults",
    "configure_ros_sensor_rates",
    "configure_mavros_rcout_policy",
    "log_ros_sensor_rates",
    "configure_mavros_battery_defaults",
    "configure_mavros_replay_state",
    "configure_mavros_state_and_rates",
]
