"""Scalar math helpers used by ROS2 bridge contracts."""

from __future__ import annotations

import numpy as np


def finite_or_zero(value: float) -> float:
    return float(value) if np.isfinite(value) else 0.0


def wrap_angle_rad(angle_rad: float) -> float:
    angle = float(angle_rad)
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle
