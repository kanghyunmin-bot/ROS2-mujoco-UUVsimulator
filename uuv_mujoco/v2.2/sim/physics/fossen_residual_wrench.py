"""Fossen-style residual hydrodynamic wrench evaluation."""

from __future__ import annotations

import numpy as np

from sim.physics.fossen_residual_named_damping import apply_named_damping
from sim.physics.fossen_residual_quadratic_damping import apply_quadratic_damping
from sim.physics.fossen_residual_terms import FOSSEN_OUTPUT_INDEX, fossen_velocity_terms


def fossen_residual_wrench_body(
    rel_lin_vel_body: np.ndarray,
    ang_vel_body: np.ndarray,
    *,
    linear: dict[str, float],
    forward_speed: dict[str, float],
    quadratic: dict[str, float],
) -> np.ndarray:
    """Compute the body-frame residual Fossen damping wrench.

    Coefficient keys follow the local convention used in sim_profiles.json:
    linear terms use ``out_src`` and quadratic terms use
    ``out_abs_absSrc_signedSrc``.
    """
    nu_terms = fossen_velocity_terms(rel_lin_vel_body, ang_vel_body)
    wrench_body = np.zeros(6, dtype=np.float64)

    apply_named_damping(wrench_body, nu_terms, FOSSEN_OUTPUT_INDEX, linear)
    apply_named_damping(
        wrench_body,
        nu_terms,
        FOSSEN_OUTPUT_INDEX,
        forward_speed,
        forward_speed_scale=abs(nu_terms["u"]),
    )
    apply_quadratic_damping(wrench_body, nu_terms, FOSSEN_OUTPUT_INDEX, quadratic)
    return wrench_body


__all__ = ["fossen_residual_wrench_body"]
