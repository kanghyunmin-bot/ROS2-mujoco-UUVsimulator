"""MuJoCo truth-state helpers for SitlTransport."""

from __future__ import annotations

import numpy as np

from bridge.sitl_transport_model_sensors import _sensor_slice, sitl_rangefinder_from_model
from bridge.sitl_transport_model_vertical import (
    estimate_base_velocity_enu,
    estimate_vertical_state,
    pressure_abs_from_depth_m,
)


def base_pos_world(self, data: mujoco.MjData) -> np.ndarray | None:
    if self._base_id < 0:
        return None
    pos_world = np.array(data.xpos[self._base_id], dtype=np.float64)
    if not np.all(np.isfinite(pos_world)):
        return None
    return pos_world


__all__ = [
    "_sensor_slice",
    "base_pos_world",
    "estimate_base_velocity_enu",
    "estimate_vertical_state",
    "pressure_abs_from_depth_m",
    "sitl_rangefinder_from_model",
]
