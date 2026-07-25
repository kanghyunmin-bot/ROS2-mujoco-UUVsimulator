"""Specific-force helpers for Ros2Bridge state estimation."""

from __future__ import annotations

import numpy as np


def clipped_specific_force(self, vec: np.ndarray) -> np.ndarray:
    return np.nan_to_num(
        np.clip(vec, -self._imu_acc_clip_mps2, self._imu_acc_clip_mps2),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )


def apply_sitl_accel_sign_contract(self, vec: np.ndarray) -> np.ndarray:
    acc = np.asarray(vec, dtype=np.float64).copy()
    acc[0] *= self._sitl_accel_xy_sign
    acc[1] *= self._sitl_accel_xy_sign
    acc[2] *= self._sitl_accel_z_sign
    return acc


def mujoco_acc_sensor_specific_force(self, acc_sensor_bmj: np.ndarray | None) -> np.ndarray | None:
    if self._imu_accel_source != "mujoco_sensor" or acc_sensor_bmj is None:
        return None
    acc_sensor_bmj = np.asarray(acc_sensor_bmj, dtype=np.float64)
    if acc_sensor_bmj.shape == (3,) and np.all(np.isfinite(acc_sensor_bmj)):
        return clipped_specific_force(self, apply_sitl_accel_sign_contract(self, acc_sensor_bmj))
    return None


def _specific_force_body(
    self,
    data: mujoco.MjData,
    acc_sensor_bmj: np.ndarray | None,
    base_vel_enu: np.ndarray,
    sim_t: float,
) -> np.ndarray | None:
    if self._base_id < 0:
        return None
    try:
        base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
    except Exception:
        return None

    sensor_force = mujoco_acc_sensor_specific_force(self, acc_sensor_bmj)
    if sensor_force is not None:
        return sensor_force

    gravity_bmj = base_rot_enu.T @ self._gravity_enu
    lin_acc_enu = self._estimate_base_accel_enu(sim_t, base_vel_enu)
    lin_acc_bmj = base_rot_enu.T @ lin_acc_enu
    specific_force_bmj = lin_acc_bmj - gravity_bmj
    specific_force_bmj = apply_sitl_accel_sign_contract(self, specific_force_bmj)
    return clipped_specific_force(self, specific_force_bmj)


__all__ = [
    "_specific_force_body",
    "apply_sitl_accel_sign_contract",
    "clipped_specific_force",
    "mujoco_acc_sensor_specific_force",
]
