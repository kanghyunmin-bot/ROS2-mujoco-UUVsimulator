"""Typed SITL sensor snapshot records for the ROS2 bridge."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sitl_types import VerticalEstimate


@dataclass(frozen=True)
class BaseKinematicState:
    sim_t: float
    base_pos_enu: np.ndarray
    base_rot_enu: np.ndarray
    quat_base: np.ndarray
    base_vel_enu: np.ndarray
    zero_vertical_reason: str | None


@dataclass(frozen=True)
class ImuDvlState:
    gyro_bmj: np.ndarray | None
    acc_bmj: np.ndarray | None
    dvl_vel_body_bmj: np.ndarray | None
    dvl_altitude_m: float | None


@dataclass(frozen=True)
class Bar30VerticalState:
    bar30_pos_enu: np.ndarray
    bar30_vel_enu: np.ndarray
    vertical_estimate: VerticalEstimate
    bar30_pressure_pa: float
    ros_depth_m: float


@dataclass(frozen=True)
class Ros2SensorSnapshot:
    base_pos_enu: np.ndarray
    base_rot_enu: np.ndarray
    quat_base: np.ndarray
    base_vel_enu: np.ndarray
    gyro_bmj: np.ndarray | None
    acc_bmj: np.ndarray | None
    dvl_vel_body_bmj: np.ndarray | None
    dvl_altitude_m: float | None
    bar30_pressure_pa: float
    ros_depth_m: float
    imu_sensor_deliveries: tuple[object, ...] = ()
    bar30_sensor_deliveries: tuple[object, ...] = ()


__all__ = [
    "Bar30VerticalState",
    "BaseKinematicState",
    "ImuDvlState",
    "Ros2SensorSnapshot",
]
