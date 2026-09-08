"""Runtime builder for Fossen residual damping and added mass."""

from __future__ import annotations

from typing import Any, Callable, Optional

from sim.physics.fossen_residual_added_mass import added_mass_active, added_mass_matrix
from sim.physics.fossen_residual_coefficients import coeff_group, nonzero_terms
from sim.physics.fossen_residual_config import log_message
from sim.physics.fossen_residual_runtime_flags import fossen_runtime_active
from sim.physics.fossen_residual_types import (
    FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS,
    FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS,
    FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS,
    FOSSEN_RESIDUAL_LINEAR_KEYS,
    FOSSEN_RESIDUAL_QUADRATIC_KEYS,
    FossenResidualRuntime,
)


def build_fossen_residual_runtime(
    sim_profile: dict[str, Any],
    *,
    use_custom_hydrodynamics: bool,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    log: Optional[Callable[[str], None]] = None,
) -> FossenResidualRuntime:
    """Build Fossen residual damping and added-mass runtime coefficients."""
    cfg = sim_profile.get("fossen_residual_hydro", {})
    if not isinstance(cfg, dict):
        cfg = {}

    linear = coeff_group(cfg, "linear", FOSSEN_RESIDUAL_LINEAR_KEYS, env_float)
    forward_speed = coeff_group(cfg, "forward_speed_linear", FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS, env_float)
    quadratic = coeff_group(cfg, "quadratic", FOSSEN_RESIDUAL_QUADRATIC_KEYS, env_float)
    added_mass_diag = coeff_group(cfg, "added_mass", FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS, env_float)
    added_mass_coupling = coeff_group(cfg, "added_mass", FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS, env_float)

    matrix = added_mass_matrix(added_mass_diag, added_mass_coupling)
    cfg_active = bool(cfg.get("active", False))
    mass_active = added_mass_active(cfg, cfg_active, matrix, env_flag)
    active = fossen_runtime_active(cfg_active, mass_active, linear, forward_speed, quadratic)
    requested_active = bool(active)
    requested_mass_active = bool(mass_active)
    if active and use_custom_hydrodynamics:
        active = False
        mass_active = False
        log_message(
            log,
            "[physics] ignoring fossen_residual_hydro in custom hydrodynamics mode; "
            "legacy Python 6DOF damping owns the Fossen-style wrench",
        )
    if active:
        log_message(
            log,
            "[physics] Fossen residual hydro active: "
            + ", ".join(
                nonzero_terms(
                    linear,
                    forward_speed,
                    quadratic,
                    added_mass_diag,
                    added_mass_coupling,
                    mass_active,
                )
            ),
        )

    return FossenResidualRuntime(
        active=bool(active),
        added_mass_active=bool(mass_active),
        requested_active=requested_active,
        requested_added_mass_active=requested_mass_active,
        linear=linear,
        forward_speed=forward_speed,
        quadratic=quadratic,
        added_mass_diag=added_mass_diag,
        added_mass_coupling=added_mass_coupling,
        added_mass_matrix=matrix,
    )


__all__ = ["build_fossen_residual_runtime"]
