"""Error math helpers for real-start runtime status payloads."""

from __future__ import annotations

import math

import numpy as np

from .pose_math import angle_error_rad


def vector_error_or_inf(value, target: np.ndarray) -> float:
    if value is None:
        return math.inf
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape == (3,) and np.all(np.isfinite(vector)) and np.all(np.isfinite(target)):
        return float(np.linalg.norm(vector - target))
    return math.inf


def xy_error_or_nan(base_xy_m: np.ndarray, *, target_x: float, target_y: float) -> float:
    base_xy_now = np.asarray(base_xy_m, dtype=np.float64)
    if math.isfinite(target_x) and math.isfinite(target_y):
        return float(np.linalg.norm(base_xy_now - np.array([target_x, target_y], dtype=np.float64)))
    return math.nan


def depth_now_for_contract(*, depth_contract: str, base_depth_m: float, bar30_depth_m: float) -> float:
    return float(base_depth_m if depth_contract == "base_link" else bar30_depth_m)


def scalar_error_or_nan(value: float, target: float) -> float:
    return float(value - target) if math.isfinite(value) and math.isfinite(target) else math.nan


def attitude_error_or_nan(
    current_rpy_rad: tuple[float, float, float],
    target_rpy: tuple[float, float, float],
) -> float:
    if all(math.isfinite(v) for v in target_rpy):
        return float(max(angle_error_rad(a, b) for a, b in zip(current_rpy_rad, target_rpy)))
    return math.nan


__all__ = [
    "attitude_error_or_nan",
    "depth_now_for_contract",
    "scalar_error_or_nan",
    "vector_error_or_inf",
    "xy_error_or_nan",
]
