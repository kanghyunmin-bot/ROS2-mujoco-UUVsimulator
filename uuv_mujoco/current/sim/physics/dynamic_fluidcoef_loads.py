"""Velocity-to-load helper math for dynamic MuJoCo fluidcoef."""

from __future__ import annotations

import math

import numpy as np


def axis_weighted_load(weights: np.ndarray, loads: np.ndarray) -> float:
    """Return the maximum weighted normalized load for one coefficient row."""
    weights = np.asarray(weights, dtype=np.float64)
    loads = np.asarray(loads, dtype=np.float64)
    if weights.size != 3 or loads.size != 3 or not np.any(weights > 0.0):
        return 0.0
    return float(np.clip(np.max(weights * loads), 0.0, 1.0))


def fluidcoef_loads_from_local_velocity(
    rel: np.ndarray,
    omega: np.ndarray,
    axis_weights: np.ndarray,
    angular_axis_weights: np.ndarray,
    *,
    reference_speed_mps: float,
    reference_angular_rps: float,
) -> np.ndarray:
    """Map local translational/angular velocity to MuJoCo fluidcoef load factors."""
    rel = np.asarray(rel, dtype=np.float64)
    omega = np.asarray(omega, dtype=np.float64)
    ref_speed = max(float(reference_speed_mps), 1.0e-9)
    ref_angular = max(float(reference_angular_rps), 1.0e-9)
    axis_loads = np.clip(np.abs(rel) / ref_speed, 0.0, 1.0)
    angular_loads = np.clip(np.abs(omega) / ref_angular, 0.0, 1.0)
    axis_weights = np.asarray(axis_weights, dtype=np.float64).reshape(5, 3)
    angular_axis_weights = np.asarray(angular_axis_weights, dtype=np.float64).reshape(5, 3)

    translational = np.array(
        [axis_weighted_load(axis_weights[row, :], axis_loads) for row in range(5)],
        dtype=np.float64,
    )
    rotational = np.array(
        [axis_weighted_load(angular_axis_weights[row, :], angular_loads) for row in range(5)],
        dtype=np.float64,
    )

    # MuJoCo fluidcoef order: blunt, slender, angular, Kutta, Magnus.
    return np.array(
        [
            translational[0],
            translational[1],
            max(translational[2], rotational[2]),
            translational[3],
            (
                math.sqrt(max(translational[4] * rotational[4], 0.0))
                if translational[4] > 0.0 and rotational[4] > 0.0
                else max(translational[4], rotational[4])
            ),
        ],
        dtype=np.float64,
    )
