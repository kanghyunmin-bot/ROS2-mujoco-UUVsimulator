"""Damping and added-mass vector assembly for hydrodynamics profiles."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .sim_profile_parsing import clamp, vector_from_keys


@dataclass(frozen=True)
class DampingConfigParts:
    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray
    air_linear_damping_diag: np.ndarray


def build_damping_config(
    sim_profile: Mapping[str, Any],
    *,
    ellipsoid_defaults: dict[str, Any] | None,
    linear_drag: float,
    angular_drag: float,
) -> DampingConfigParts:
    defaults = _linear_angular_defaults(ellipsoid_defaults, linear_drag, angular_drag)
    (
        default_linear_linear,
        default_linear_angular,
        default_quadratic_linear,
        default_quadratic_angular,
        default_added_mass_linear,
        default_added_mass_angular,
    ) = defaults
    return DampingConfigParts(
        added_mass_diag=vector_from_keys(
            sim_profile,
            direct_key="added_mass_diag",
            split_prefix="added_mass",
            default_linear=default_added_mass_linear,
            default_angular=default_added_mass_angular,
        ),
        linear_damping_diag=vector_from_keys(
            sim_profile,
            direct_key="linear_damping_diag",
            split_prefix="linear_damping",
            default_linear=default_linear_linear,
            default_angular=default_linear_angular,
        ),
        quadratic_damping_diag=vector_from_keys(
            sim_profile,
            direct_key="quadratic_damping_diag",
            split_prefix="quadratic_damping",
            default_linear=default_quadratic_linear,
            default_angular=default_quadratic_angular,
        ),
        air_linear_damping_diag=air_damping_diag(sim_profile, linear_drag, angular_drag),
    )


def _linear_angular_defaults(
    ellipsoid_defaults: dict[str, Any] | None,
    linear_drag: float,
    angular_drag: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if ellipsoid_defaults is None:
        return (
            np.full(3, linear_drag, dtype=np.float64),
            np.full(3, angular_drag, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
        )

    return (
        np.array(ellipsoid_defaults["linear_damping_diag"][:3], dtype=np.float64),
        np.array(ellipsoid_defaults["linear_damping_diag"][3:], dtype=np.float64),
        np.array(ellipsoid_defaults["quadratic_damping_diag"][:3], dtype=np.float64),
        np.array(ellipsoid_defaults["quadratic_damping_diag"][3:], dtype=np.float64),
        np.array(ellipsoid_defaults["added_mass_diag"][:3], dtype=np.float64),
        np.array(ellipsoid_defaults["added_mass_diag"][3:], dtype=np.float64),
    )


def air_damping_diag(sim_profile: Mapping[str, Any], linear_drag: float, angular_drag: float) -> np.ndarray:
    return np.concatenate(
        (
            np.full(3, clamp(float(sim_profile.get("air_linear_drag", linear_drag * 0.03)), 0.0, linear_drag)),
            np.full(3, clamp(float(sim_profile.get("air_angular_drag", angular_drag * 0.05)), 0.0, angular_drag)),
        )
    ).astype(np.float64, copy=False)


__all__ = ["DampingConfigParts", "air_damping_diag", "build_damping_config"]
