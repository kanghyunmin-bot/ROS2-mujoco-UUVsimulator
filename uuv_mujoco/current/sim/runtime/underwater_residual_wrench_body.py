"""Body-frame residual hydrodynamic wrench calculations."""

from __future__ import annotations

from typing import Any

import numpy as np

from physics.hydrodynamics_helpers import added_mass_coriolis
from sim.physics.cfd_dynamic_wrench import cfd_dynamic_force_body
from sim.physics.fossen_residual import fossen_residual_wrench_body
from sim.runtime.underwater_wrench_apply import immersed_fraction


def residual_hydro_force_torque_body(hyd: Any, rel_lin_vel_body: np.ndarray, ang_vel_body: np.ndarray):
    _, v_body, w_body = rel_lin_vel_body
    _, q_body, r_body = ang_vel_body
    residual_force_body = np.array(
        [
            0.0,
            -hyd.residual_hydro_coeffs["y_v"] * float(v_body)
            - hyd.residual_hydro_coeffs["y_r"] * float(r_body),
            -hyd.residual_hydro_coeffs["z_w"] * float(w_body)
            - hyd.residual_hydro_coeffs["z_q"] * float(q_body),
        ],
        dtype=np.float64,
    )
    residual_torque_body = np.array(
        [
            0.0,
            -hyd.residual_hydro_coeffs["m_w"] * float(w_body)
            - hyd.residual_hydro_coeffs["m_q"] * float(q_body),
            -hyd.residual_hydro_coeffs["n_v"] * float(v_body)
            - hyd.residual_hydro_coeffs["n_r"] * float(r_body),
        ],
        dtype=np.float64,
    )
    return residual_force_body, residual_torque_body


def fossen_added_mass_wrench_body(hyd: Any, nu_rel_body: np.ndarray, rel_acc_body: np.ndarray, submerged: float):
    immersed_matrix = hyd.fossen_residual_added_mass_matrix * immersed_fraction(submerged)
    added_mass_wrench_body = -(immersed_matrix @ rel_acc_body)
    added_mass_wrench_body -= added_mass_coriolis(immersed_matrix, nu_rel_body) @ nu_rel_body
    return added_mass_wrench_body


def fossen_residual_wrench_from_hyd(hyd: Any, rel_lin_vel_body: np.ndarray, ang_vel_body: np.ndarray):
    return fossen_residual_wrench_body(
        rel_lin_vel_body,
        ang_vel_body,
        linear=hyd.fossen_residual_linear,
        forward_speed=hyd.fossen_residual_forward_speed,
        quadratic=hyd.fossen_residual_quadratic,
    )


def cfd_dynamic_force_from_hyd(hyd: Any, rel_lin_vel_body: np.ndarray):
    return cfd_dynamic_force_body(
        rel_lin_vel_body,
        hyd.cfd_dynamic_wrench_axes,
        scale=hyd.cfd_dynamic_wrench_scale,
    )


__all__ = [
    "cfd_dynamic_force_from_hyd",
    "fossen_added_mass_wrench_body",
    "fossen_residual_wrench_from_hyd",
    "residual_hydro_force_torque_body",
]
