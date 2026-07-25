"""Coefficient extraction and reporting for Fossen residual hydrodynamics."""

from __future__ import annotations

from typing import Any, Callable

from sim.physics.fossen_residual_config import config_group
from sim.physics.fossen_residual_types import RESIDUAL_HYDRO_KEYS


def coeff_group(
    cfg: dict[str, Any],
    group: str,
    keys: tuple[str, ...],
    env_float: Callable[[str, float], float],
) -> dict[str, float]:
    values = config_group(cfg, group)

    def coeff(name: str) -> float:
        env_name = f"UUV_FOSSEN_RES_{group.upper()}_{name.upper()}"
        return env_float(env_name, float(values.get(name, 0.0)))

    return {key: coeff(key) for key in keys}


def residual_hydro_coefficients(
    cfg: dict[str, Any],
    env_float: Callable[[str, float], float],
) -> dict[str, float]:
    linear_cfg = config_group(cfg, "linear")

    def coeff(name: str) -> float:
        env_name = f"UUV_HYDRO_RES_{name.upper()}"
        return env_float(env_name, float(linear_cfg.get(name, 0.0)))

    return {key: coeff(key) for key in RESIDUAL_HYDRO_KEYS}


def nonzero_terms(
    linear: dict[str, float],
    forward_speed: dict[str, float],
    quadratic: dict[str, float],
    added_mass_diag: dict[str, float],
    added_mass_coupling: dict[str, float],
    added_mass_active: bool,
) -> list[str]:
    terms = []
    for group_name, group_values in (
        ("linear", linear),
        ("forward_speed_linear", forward_speed),
        ("quadratic", quadratic),
    ):
        terms.extend(
            f"{group_name}.{name}={value:+.4f}"
            for name, value in group_values.items()
            if abs(value) > 1.0e-9
        )
    if added_mass_active:
        for name, value in {**added_mass_diag, **added_mass_coupling}.items():
            if abs(value) > 1.0e-9:
                terms.append(f"added_mass.{name}={value:+.4f}")
    return terms


__all__ = ["coeff_group", "nonzero_terms", "residual_hydro_coefficients"]
