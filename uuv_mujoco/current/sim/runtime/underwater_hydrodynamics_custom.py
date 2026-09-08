"""Custom diagonal hydrodynamic damping and added-mass application."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import added_mass_coriolis


def apply_custom_hydrodynamics(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    lin_vel_body: np.ndarray,
    rel_lin_vel_world: np.ndarray,
    nu_rel_body: np.ndarray,
    rel_acc_body: np.ndarray,
    coefficient_scales,
    submerged: float,
) -> None:
    hyd = runtime.hydrodynamics
    data = runtime.data
    base_id = int(runtime.base_id)
    surface_weight = max(0.0, 4.0 * submerged * (1.0 - submerged))
    if hyd.surface_heave_damping > 1e-9 and surface_weight > 1e-9:
        surface_force_world = np.array(
            [0.0, 0.0, -hyd.surface_heave_damping * surface_weight * float(rel_lin_vel_world[2])],
            dtype=np.float64,
        )
        data.xfrc_applied[base_id, 0:3] += surface_force_world

    rel_flow_world = hyd.water_current_world - (base_rot @ lin_vel_body)
    immersed_added_mass = hyd.added_mass_diag * coefficient_scales.added_mass_diag * submerged
    immersed_linear_damping = (
        hyd.linear_damping_diag * coefficient_scales.linear_damping_diag * submerged
    )
    immersed_quadratic_damping = (
        hyd.quadratic_damping_diag * coefficient_scales.quadratic_damping_diag * submerged
    )
    immersed_linear_damping[2] *= hyd.heave_damping_scale
    immersed_quadratic_damping[2] *= hyd.heave_damping_scale

    hydro_wrench_body = np.zeros(6, dtype=np.float64)
    if np.any(immersed_added_mass > 1e-9):
        hydro_wrench_body -= immersed_added_mass * rel_acc_body
        hydro_wrench_body -= added_mass_coriolis(immersed_added_mass, nu_rel_body) @ nu_rel_body
    hydro_wrench_body -= immersed_linear_damping * nu_rel_body
    hydro_wrench_body -= immersed_quadratic_damping * np.abs(nu_rel_body) * nu_rel_body

    data.xfrc_applied[base_id, 0:3] += base_rot @ hydro_wrench_body[:3]
    data.xfrc_applied[base_id, 3:6] += base_rot @ hydro_wrench_body[3:]
    runtime.last_flow_world = rel_flow_world


__all__ = ["apply_custom_hydrodynamics"]
