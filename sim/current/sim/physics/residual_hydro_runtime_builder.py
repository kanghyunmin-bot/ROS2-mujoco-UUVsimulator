"""Runtime builder for the lightweight residual hydro wrench."""

from __future__ import annotations

from typing import Any, Callable, Optional

from sim.physics.fossen_residual_coefficients import residual_hydro_coefficients
from sim.physics.fossen_residual_config import log_message
from sim.physics.fossen_residual_runtime_flags import residual_hydro_active
from sim.physics.fossen_residual_types import ResidualHydroRuntime


def build_residual_hydro_runtime(
    sim_profile: dict[str, Any],
    *,
    use_custom_hydrodynamics: bool,
    env_float: Callable[[str, float], float],
    log: Optional[Callable[[str], None]] = None,
) -> ResidualHydroRuntime:
    """Build the lightweight residual hydro wrench coefficient contract."""
    cfg = sim_profile.get("hydro_residual_wrench", {})
    if not isinstance(cfg, dict):
        cfg = {}

    coeffs = residual_hydro_coefficients(cfg, env_float)
    active = residual_hydro_active(bool(cfg.get("active", False)), coeffs)
    if active and use_custom_hydrodynamics:
        active = False
        log_message(
            log,
            "[physics] ignoring hydro_residual_wrench in custom hydrodynamics mode; "
            "legacy Python 6DOF damping owns residual terms",
        )
    if active:
        coeff_text = ", ".join(f"{name}={value:+.4f}" for name, value in coeffs.items())
        log_message(log, "[physics] residual hydro wrench active: " f"{coeff_text}")
    return ResidualHydroRuntime(active=active, coeffs=coeffs)


__all__ = ["build_residual_hydro_runtime"]
