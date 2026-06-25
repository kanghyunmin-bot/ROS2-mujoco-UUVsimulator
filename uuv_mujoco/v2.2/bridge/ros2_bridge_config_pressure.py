"""Pressure, IMU, and vertical feedback configuration facade."""

from __future__ import annotations

from .ros2_bridge_config_baro import configure_baro_pressure_contract
from .ros2_bridge_config_imu import configure_imu_accel_contract
from .ros2_bridge_config_vertical import configure_vertical_feedback_contract


def configure_pressure_vertical_contract(bridge: object) -> None:
    configure_baro_pressure_contract(bridge)
    configure_imu_accel_contract(bridge)
    configure_vertical_feedback_contract(bridge)


__all__ = [
    "configure_baro_pressure_contract",
    "configure_imu_accel_contract",
    "configure_pressure_vertical_contract",
    "configure_vertical_feedback_contract",
]
