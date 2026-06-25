"""Hydrodynamic underwater wrench runtime helpers."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_custom_dispatch import apply_or_clear_custom_hydrodynamics
from sim.runtime.underwater_hydrodynamics_extra import apply_empirical_pitch_lift_heave
from sim.runtime.underwater_relative_acceleration import update_relative_acceleration
from sim.runtime.underwater_residual_dispatch import apply_enabled_residual_wrenches


def apply_hydrodynamic_wrenches(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    lin_vel_body: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    rel_lin_vel_world: np.ndarray,
    ang_vel_body: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    submerged: float,
    buoyancy_submerged: float,
) -> None:
    apply_or_clear_custom_hydrodynamics(
        runtime,
        base_rot=base_rot,
        lin_vel_body=lin_vel_body,
        rel_lin_vel_world=rel_lin_vel_world,
        nu_rel_body=nu_rel_body,
        rel_acc_body=rel_acc_body,
        submerged=submerged,
    )
    apply_enabled_residual_wrenches(
        runtime,
        base_rot=base_rot,
        rel_lin_vel_body=rel_lin_vel_body,
        ang_vel_body=ang_vel_body,
        nu_rel_body=nu_rel_body,
        rel_acc_body=rel_acc_body,
        submerged=submerged,
    )
    apply_empirical_pitch_lift_heave(
        runtime,
        base_rot=base_rot,
        rel_lin_vel_body=rel_lin_vel_body,
        buoyancy_submerged=buoyancy_submerged,
    )


__all__ = ["update_relative_acceleration", "apply_hydrodynamic_wrenches"]
