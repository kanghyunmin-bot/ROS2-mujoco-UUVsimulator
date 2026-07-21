"""DVL velocity helpers for Ros2Bridge state estimation."""

from __future__ import annotations

import numpy as np


def _dvl_velocity_body(
    self,
    data: mujoco.MjData,
    dvl_vel_sensor: np.ndarray | None,
    _gyro_bmj: np.ndarray | None,
) -> np.ndarray | None:
    if dvl_vel_sensor is None:
        return None
    vel_body = np.array(dvl_vel_sensor, dtype=np.float64)
    if self._base_id >= 0 and self._dvl_site_id >= 0:
        try:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            dvl_rot_enu = data.site_xmat[self._dvl_site_id].reshape(3, 3).copy()
            rot_bmj_dvl = base_rot_enu.T @ dvl_rot_enu
            vel_body = rot_bmj_dvl @ vel_body
        except Exception:
            pass
    vel_body = np.nan_to_num(vel_body, nan=0.0, posinf=0.0, neginf=0.0)
    alpha = float(self._dvl_filter_alpha)
    if alpha <= 0.0:
        self._dvl_vel_body_filt = vel_body
        return vel_body
    if self._dvl_vel_body_filt is None:
        self._dvl_vel_body_filt = vel_body
    else:
        self._dvl_vel_body_filt = ((1.0 - alpha) * self._dvl_vel_body_filt) + (alpha * vel_body)
    return self._dvl_vel_body_filt.copy()


__all__ = ["_dvl_velocity_body"]
