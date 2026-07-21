"""Apply empirical residual hydrodynamic force and torque."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_residual_wrench_body import residual_hydro_force_torque_body
from sim.runtime.underwater_wrench_apply import apply_body_force_torque


def apply_residual_hydro(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    ang_vel_body: np.ndarray,
    submerged: float,
) -> None:
    residual_force_body, residual_torque_body = residual_hydro_force_torque_body(
        runtime.hydrodynamics,
        rel_lin_vel_body,
        ang_vel_body,
    )
    apply_body_force_torque(
        data=runtime.data,
        base_id=int(runtime.base_id),
        base_rot=base_rot,
        force_body=residual_force_body,
        torque_body=residual_torque_body,
        submerged=submerged,
    )


__all__ = ["apply_residual_hydro"]
