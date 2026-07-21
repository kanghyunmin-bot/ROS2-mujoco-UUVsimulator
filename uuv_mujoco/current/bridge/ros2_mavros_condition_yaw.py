"""COMMAND_LONG CONDITION_YAW handling for the ROS2 MAVROS facade."""

from __future__ import annotations

import time

import numpy as np


def apply_condition_yaw_command(self, request) -> None:
    angle_deg = float(getattr(request, "param1", 0.0))
    is_relative = float(getattr(request, "param4", 0.0))
    direction = float(getattr(request, "param3", 0.0))
    direction = 1.0 if direction >= 0.0 else -1.0
    delta_rad = np.deg2rad(angle_deg * direction)
    if is_relative > 0.5:
        self._mavros_setpoint_yaw = None
        self._mavros_pending_yaw_delta += float(delta_rad)
    else:
        self._mavros_setpoint_yaw = float(np.deg2rad(angle_deg))
        self._mavros_pending_yaw_delta = 0.0
    self._mavros_setpoint_last_t = time.monotonic()


__all__ = ["apply_condition_yaw_command"]
