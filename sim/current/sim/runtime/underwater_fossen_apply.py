"""Apply Fossen residual hydrodynamic wrench families."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_residual_wrench_body import (
    fossen_added_mass_wrench_body,
    fossen_residual_wrench_from_hyd,
)
from sim.runtime.underwater_wrench_apply import apply_body_wrench


def apply_fossen_added_mass_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    submerged: float,
) -> None:
    added_mass_wrench_body = fossen_added_mass_wrench_body(
        runtime.hydrodynamics,
        nu_rel_body,
        rel_acc_body,
        submerged,
    )
    apply_body_wrench(
        data=runtime.data,
        base_id=int(runtime.base_id),
        base_rot=base_rot,
        wrench_body=added_mass_wrench_body,
        submerged=1.0,
    )


def apply_fossen_residual_wrench(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    ang_vel_body: np.ndarray,
    submerged: float,
) -> None:
    fossen_wrench_body = fossen_residual_wrench_from_hyd(
        runtime.hydrodynamics,
        rel_lin_vel_body,
        ang_vel_body,
    )
    apply_body_wrench(
        data=runtime.data,
        base_id=int(runtime.base_id),
        base_rot=base_rot,
        wrench_body=fossen_wrench_body,
        submerged=submerged,
    )


__all__ = ["apply_fossen_added_mass_wrench", "apply_fossen_residual_wrench"]
