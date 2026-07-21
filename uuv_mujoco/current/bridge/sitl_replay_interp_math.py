"""Numeric helpers for SITL sensor replay interpolation."""

from __future__ import annotations

import numpy as np


def lerp_scalar(left: float, right: float, alpha: float) -> float:
    return float((1.0 - alpha) * float(left) + alpha * float(right))


def lerp_vec(left: np.ndarray, right: np.ndarray, alpha: float) -> np.ndarray:
    return (1.0 - alpha) * np.asarray(left, dtype=np.float64) + alpha * np.asarray(
        right, dtype=np.float64
    )


def shortest_quat_pair(left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    right_aligned = np.asarray(right, dtype=np.float64)
    if float(np.dot(left, right_aligned)) < 0.0:
        right_aligned = -right_aligned
    return np.asarray(left, dtype=np.float64), right_aligned


__all__ = ["lerp_scalar", "lerp_vec", "shortest_quat_pair"]
