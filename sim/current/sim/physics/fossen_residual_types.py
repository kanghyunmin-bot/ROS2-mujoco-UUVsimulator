"""Types and coefficient key contracts for Fossen residual hydrodynamics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


RESIDUAL_HYDRO_KEYS = (
    "y_v",
    "n_r",
    "y_r",
    "n_v",
    "z_w",
    "m_q",
    "z_q",
    "m_w",
)

FOSSEN_RESIDUAL_LINEAR_KEYS = (
    "y_v",
    "y_r",
    "n_v",
    "n_r",
    "z_w",
    "z_q",
    "m_w",
    "m_q",
)

FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS = FOSSEN_RESIDUAL_LINEAR_KEYS

FOSSEN_RESIDUAL_QUADRATIC_KEYS = (
    "x_abs_u_u",
    "y_abs_v_v",
    "n_abs_r_r",
    "z_abs_w_w",
    "m_abs_q_q",
    "y_abs_r_r",
    "n_abs_v_v",
    "z_abs_q_q",
    "m_abs_w_w",
)

FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS = (
    "x_u",
    "y_v",
    "z_w",
    "k_p",
    "m_q",
    "n_r",
)

FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS = (
    "y_r_n_v",
    "z_q_m_w",
)


@dataclass(frozen=True)
class ResidualHydroRuntime:
    active: bool
    coeffs: dict[str, float]


@dataclass(frozen=True)
class FossenResidualRuntime:
    active: bool
    added_mass_active: bool
    linear: dict[str, float]
    forward_speed: dict[str, float]
    quadratic: dict[str, float]
    added_mass_diag: dict[str, float]
    added_mass_coupling: dict[str, float]
    added_mass_matrix: np.ndarray


__all__ = [
    "FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS",
    "FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS",
    "FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS",
    "FOSSEN_RESIDUAL_LINEAR_KEYS",
    "FOSSEN_RESIDUAL_QUADRATIC_KEYS",
    "FossenResidualRuntime",
    "RESIDUAL_HYDRO_KEYS",
    "ResidualHydroRuntime",
]
