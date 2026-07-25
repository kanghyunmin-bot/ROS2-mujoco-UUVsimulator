"""MuJoCo sensor slicing helpers for SitlTransport."""

from __future__ import annotations

import numpy as np


def _sensor_slice(self, sensor_id: int, data: mujoco.MjData) -> np.ndarray | None:
    if sensor_id < 0:
        return None
    adr = int(self.model.sensor_adr[sensor_id])
    dim = int(self.model.sensor_dim[sensor_id])
    return data.sensordata[adr : adr + dim]


def sitl_rangefinder_from_model(self, data: mujoco.MjData) -> float | None:
    """Return downward-looking DVL altitude as a JSON rangefinder distance."""
    dvl_alt = self._sensor_slice(self._dvl_altitude_sensor_id, data)
    if dvl_alt is None or len(dvl_alt) < 1:
        return None
    distance_m = float(dvl_alt[0])
    if not np.isfinite(distance_m):
        return None
    if distance_m < 0.0 or distance_m > self._sitl_rangefinder_max_m:
        return None
    return distance_m


__all__ = ["_sensor_slice", "sitl_rangefinder_from_model"]
