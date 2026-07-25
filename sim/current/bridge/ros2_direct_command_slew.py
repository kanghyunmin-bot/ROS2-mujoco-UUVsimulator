"""Deadband and slew helpers for direct MuJoCo command callbacks."""

from __future__ import annotations

import numpy as np

from .ros2_math import finite_or_zero


def apply_cmd_deadband(value: float, deadband_norm: float) -> float:
    value = finite_or_zero(value)
    if abs(value) <= deadband_norm:
        return 0.0
    return float(np.clip(value, -1.0, 1.0))


def raw_command_vector(bridge, fwd_norm: float, sway_norm: float, yaw_norm: float, heave_norm: float) -> np.ndarray:
    return np.array(
        [
            apply_cmd_deadband(fwd_norm, bridge._cmd_deadband_norm),
            apply_cmd_deadband(sway_norm, bridge._cmd_deadband_norm),
            apply_cmd_deadband(yaw_norm, bridge._cmd_deadband_norm),
            apply_cmd_deadband(heave_norm, bridge._cmd_deadband_norm),
        ],
        dtype=np.float64,
    )


def slew_command_vector(previous: np.ndarray, raw: np.ndarray, *, dt: float, slew_rate_norm: float) -> np.ndarray:
    delta = raw - previous
    max_delta = slew_rate_norm * dt
    return previous + np.clip(delta, -max_delta, max_delta)


__all__ = ["apply_cmd_deadband", "raw_command_vector", "slew_command_vector"]
