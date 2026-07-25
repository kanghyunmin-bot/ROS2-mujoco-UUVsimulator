"""IMU gyro vector helpers for Ros2Bridge state estimation."""

from __future__ import annotations

import numpy as np


def _imu_vectors_in_body(self, data: mujoco.MjData, gyro: np.ndarray | None) -> np.ndarray | None:
    gyro_bmj = np.array(gyro, dtype=np.float64) if gyro is not None else None
    if self._base_id < 0 or self._imu_site_id < 0 or gyro_bmj is None:
        return gyro_bmj
    try:
        base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
        imu_rot_enu = data.site_xmat[self._imu_site_id].reshape(3, 3).copy()
        rot_bmj_imu = base_rot_enu.T @ imu_rot_enu
        return rot_bmj_imu @ gyro_bmj
    except Exception:
        return gyro_bmj


__all__ = ["_imu_vectors_in_body"]
