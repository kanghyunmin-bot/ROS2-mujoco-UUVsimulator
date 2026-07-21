"""IMU vector assembly for SITL sensor snapshots."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_state_specific_force import apply_sitl_accel_sign_contract
from .ros2_sitl_sensor_types import BaseKinematicState


def rotated_acc_sensor_body(owner: Any, data: Any, base: BaseKinematicState, acc_sensor: Any) -> np.ndarray | None:
    acc_sensor_bmj = np.array(acc_sensor, dtype=np.float64) if acc_sensor is not None else None
    if acc_sensor_bmj is not None and owner._imu_site_id >= 0:
        try:
            imu_rot_enu = data.site_xmat[owner._imu_site_id].reshape(3, 3).copy()
            rot_bmj_imu = base.base_rot_enu.T @ imu_rot_enu
            acc_sensor_bmj = rot_bmj_imu @ acc_sensor_bmj
        except Exception:
            pass
    return acc_sensor_bmj


def scaled_accel(owner: Any, acc_bmj: Any) -> np.ndarray | None:
    if acc_bmj is None:
        return None
    acc = np.asarray(acc_bmj, dtype=np.float64).copy()
    acc[0] *= owner._imu_accel_xy_scale
    acc[1] *= owner._imu_accel_xy_scale
    acc[2] *= owner._imu_accel_z_scale
    return acc


def imu_body_vectors_from_snapshot(owner: Any, data: Any, base: BaseKinematicState, gyro: Any, acc_sensor: Any) -> tuple[np.ndarray, np.ndarray | None]:
    gyro_bmj = owner._imu_vectors_in_body(data, gyro)
    acc_sensor_bmj = rotated_acc_sensor_body(owner, data, base, acc_sensor)
    acc_bmj = scaled_accel(owner, owner._specific_force_body(data, acc_sensor_bmj, base.base_vel_enu, base.sim_t))
    if base.zero_vertical_reason:
        if owner._sitl_initial_depth_hold_active:
            gyro_bmj = np.zeros(3, dtype=np.float64)
        gravity_bmj = base.base_rot_enu.T @ owner._gravity_enu
        acc_bmj = scaled_accel(owner, apply_sitl_accel_sign_contract(owner, -gravity_bmj))
    return gyro_bmj, acc_bmj


__all__ = ["imu_body_vectors_from_snapshot", "rotated_acc_sensor_body", "scaled_accel"]
