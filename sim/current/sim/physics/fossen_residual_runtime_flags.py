"""Runtime activation gates for residual hydrodynamics contracts."""

from __future__ import annotations


def residual_hydro_active(cfg_active: bool, coeffs: dict[str, float]) -> bool:
    return bool(cfg_active and any(abs(value) > 1.0e-9 for value in coeffs.values()))


def fossen_runtime_active(
    cfg_active: bool,
    added_mass_active: bool,
    linear: dict[str, float],
    forward_speed: dict[str, float],
    quadratic: dict[str, float],
) -> bool:
    damping_active = any(
        abs(value) > 1.0e-9
        for group_values in (linear, forward_speed, quadratic)
        for value in group_values.values()
    )
    return bool((cfg_active and damping_active) or bool(added_mass_active))


__all__ = ["fossen_runtime_active", "residual_hydro_active"]
