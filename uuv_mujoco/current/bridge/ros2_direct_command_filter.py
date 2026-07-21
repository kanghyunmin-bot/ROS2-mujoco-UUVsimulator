"""Deadband, slew, and direct plant command filtering for Ros2Bridge."""

from __future__ import annotations

import time

import numpy as np

from .ros2_direct_command_guard import direct_command_blocked_by_sitl, warn_direct_command_blocked_once
from .ros2_direct_command_slew import apply_cmd_deadband, raw_command_vector, slew_command_vector


def _apply_cmd_deadband(self, value: float) -> float:
    return apply_cmd_deadband(value, self._cmd_deadband_norm)


def _handle_normalized_cmd(self, fwd_norm: float, sway_norm: float, yaw_norm: float, heave_norm: float) -> None:
    if direct_command_blocked_by_sitl(self):
        warn_direct_command_blocked_once(self)
        return
    now = time.monotonic()
    raw = raw_command_vector(self, fwd_norm, sway_norm, yaw_norm, heave_norm)
    if self._cmd_filter_t < 0.0 or self._cmd_slew_rate_norm <= 0.0:
        self._cmd_filter_norm = raw
    else:
        dt = max(0.0, now - self._cmd_filter_t)
        self._cmd_filter_norm = slew_command_vector(
            self._cmd_filter_norm,
            raw,
            dt=dt,
            slew_rate_norm=self._cmd_slew_rate_norm,
        )
    self._cmd_filter_t = now
    self.last_cmd_wall = now
    self.cmd_active = True
    self.command_callback(
        float(self._cmd_filter_norm[0] * self.cmd_limit),
        float(self._cmd_filter_norm[1] * self.cmd_limit),
        float(self._cmd_filter_norm[2] * self.cmd_limit),
        float(self._cmd_filter_norm[3] * self.cmd_limit),
    )


def _clear_cmd(self) -> None:
    self._cmd_filter_t = time.monotonic()
    self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
    if direct_command_blocked_by_sitl(self):
        self.cmd_active = False
        return
    self.command_callback(0.0, 0.0, 0.0, 0.0)
    self.cmd_active = False


__all__ = ["_apply_cmd_deadband", "_clear_cmd", "_handle_normalized_cmd"]
