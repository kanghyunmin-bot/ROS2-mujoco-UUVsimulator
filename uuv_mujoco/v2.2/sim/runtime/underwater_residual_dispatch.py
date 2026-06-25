"""Residual hydrodynamic wrench-family dispatch."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_hydrodynamics_residual import (
    apply_cfd_dynamic_wrench,
    apply_fossen_added_mass_wrench,
    apply_fossen_residual_wrench,
    apply_residual_hydro,
)


def apply_enabled_residual_wrenches(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    rel_lin_vel_body: np.ndarray,
    ang_vel_body: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    submerged: float,
) -> None:
    hyd = runtime.hydrodynamics

    if hyd.residual_hydro_active:
        apply_residual_hydro(
            runtime,
            base_rot=base_rot,
            rel_lin_vel_body=rel_lin_vel_body,
            ang_vel_body=ang_vel_body,
            submerged=submerged,
        )

    if hyd.fossen_residual_added_mass_active:
        apply_fossen_added_mass_wrench(
            runtime,
            base_rot=base_rot,
            nu_rel_body=nu_rel_body,
            rel_acc_body=rel_acc_body,
            submerged=submerged,
        )

    if hyd.fossen_residual_active:
        apply_fossen_residual_wrench(
            runtime,
            base_rot=base_rot,
            rel_lin_vel_body=rel_lin_vel_body,
            ang_vel_body=ang_vel_body,
            submerged=submerged,
        )

    if hyd.cfd_dynamic_wrench_enabled:
        apply_cfd_dynamic_wrench(
            runtime,
            base_rot=base_rot,
            rel_lin_vel_body=rel_lin_vel_body,
            submerged=submerged,
        )


__all__ = ["apply_enabled_residual_wrenches"]
