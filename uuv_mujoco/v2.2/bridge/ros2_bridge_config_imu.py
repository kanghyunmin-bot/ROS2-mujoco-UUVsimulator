"""IMU acceleration and static-pressure output configuration facade."""

from __future__ import annotations

from bridge.ros2_bridge_config_imu_ros_surface import configure_ros_imu_accel_surface_contract
from bridge.ros2_bridge_config_imu_sitl import configure_sitl_imu_accel_contract
from bridge.ros2_bridge_config_imu_source import configure_imu_accel_source
from bridge.ros2_bridge_config_static_pressure import configure_static_pressure_output_contract


def configure_imu_accel_contract(bridge: object) -> None:
    configure_imu_accel_source(bridge)
    configure_sitl_imu_accel_contract(bridge)
    configure_ros_imu_accel_surface_contract(bridge)
    configure_static_pressure_output_contract(bridge)


__all__ = [
    "configure_imu_accel_contract",
    "configure_imu_accel_source",
    "configure_ros_imu_accel_surface_contract",
    "configure_sitl_imu_accel_contract",
    "configure_static_pressure_output_contract",
]
