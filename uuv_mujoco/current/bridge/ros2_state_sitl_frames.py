"""NED frame construction for ArduSub SITL vertical state."""

from __future__ import annotations

import numpy as np


def sitl_vertical_frames(
    self,
    *,
    base_pos_enu: np.ndarray,
    base_vel_enu: np.ndarray,
    depth_m: float,
    vel_d: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pos_ned = self._enu_to_ned @ np.asarray(base_pos_enu, dtype=np.float64)
    extnav_pos_ned = pos_ned.copy()
    extnav_pos_ned[2] = float(max(0.0, self._water_surface_z - float(base_pos_enu[2])))
    vel_ned = self._enu_to_ned @ np.asarray(base_vel_enu, dtype=np.float64)
    pos_ned[2] = float(depth_m)
    vel_ned[2] = float(vel_d)
    return pos_ned, vel_ned, extnav_pos_ned


__all__ = ["sitl_vertical_frames"]
