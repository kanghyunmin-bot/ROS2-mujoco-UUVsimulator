"""Body-frame CFD residual force evaluation."""

from __future__ import annotations

import math

import numpy as np

from sim.physics.cfd_dynamic_wrench_table import cfd_force_table_lookup


def cfd_dynamic_force_body(
    rel_lin_vel_body: np.ndarray,
    axes: dict[str, dict[str, object]],
    *,
    scale: float,
) -> np.ndarray:
    """Compute body-frame CFD residual force for the configured translational axes."""
    rel_lin_vel_body = np.asarray(rel_lin_vel_body, dtype=np.float64)
    force_body = np.zeros(3, dtype=np.float64)
    for axis_cfg in axes.values():
        _accumulate_axis_force(force_body, rel_lin_vel_body, axis_cfg, scale=float(scale))
    return force_body


def _accumulate_axis_force(
    force_body: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    axis_cfg: dict[str, object],
    *,
    scale: float,
) -> None:
    axis_index = int(axis_cfg["index"])
    rel_axis_speed = float(rel_lin_vel_body[axis_index])
    if abs(rel_axis_speed) <= 1.0e-12:
        return
    magnitudes = axis_cfg["positive"] if rel_axis_speed > 0.0 else axis_cfg["negative"]
    force_mag = cfd_force_table_lookup(
        abs(rel_axis_speed),
        axis_cfg["speeds"],
        magnitudes,
        str(axis_cfg["extrapolate"]),
    )
    force_body[axis_index] = -math.copysign(scale * force_mag, rel_axis_speed)


__all__ = ["cfd_dynamic_force_body"]
