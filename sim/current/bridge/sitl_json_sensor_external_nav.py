"""ExternalNav dispatch for SITL JSON sensor packets."""

from __future__ import annotations

import numpy as np

from bridge.sitl_types import VerticalEstimate


def dispatch_json_sensor_external_nav(
    self,
    sitl_t: float,
    vertical_est: VerticalEstimate,
    quat: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    self._cache_external_nav_state(sitl_t, vertical_est, quat, roll, pitch, yaw)
    if not self.live_wall_external_nav:
        self._send_external_nav(sitl_t, vertical_est, quat, roll, pitch, yaw)
        self._enforce_extnav_contract()


__all__ = ["dispatch_json_sensor_external_nav"]
