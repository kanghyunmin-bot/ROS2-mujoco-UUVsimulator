"""Equivalent-ellipsoid 6DOF hydrodynamic coefficient assembly."""

from __future__ import annotations

from typing import Iterable

import numpy as np

from .ellipsoid_geometry import (
    ellipsoid_depolarization_factors,
    ellipsoid_projected_areas,
    ellipsoid_volume,
    positive_semi_axes,
)
from .ellipsoid_hydro_types import EllipsoidHydroEstimate


def estimate_ellipsoid_hydrodynamics(
    semi_axes: Iterable[float],
    fluid_density: float,
    *,
    effective_cd_linear: Iterable[float] = (0.10, 0.11, 0.13),
    effective_cd_angular: Iterable[float] = (2.2, 2.4, 1.8),
    added_mass_scale_linear: Iterable[float] = (1.0, 1.0, 1.0),
    added_mass_scale_angular: Iterable[float] = (1.2, 1.2, 1.0),
    linear_damping_ratio_linear: float = 2.0,
    linear_damping_ratio_angular: float = 1.0,
    reference_speed_linear: float = 0.30,
    reference_speed_angular: float = 0.75,
) -> EllipsoidHydroEstimate:
    """Build a tunable ellipsoid baseline for the Python 6DOF plant model."""

    semi = positive_semi_axes(semi_axes)
    rho = float(max(fluid_density, 1e-6))
    volume = ellipsoid_volume(semi)
    areas = ellipsoid_projected_areas(semi)
    depol = ellipsoid_depolarization_factors(semi)

    cd_linear = _clipped_vector(effective_cd_linear, 0.0, 10.0)
    cd_angular = _clipped_vector(effective_cd_angular, 0.0, 20.0)
    scale_linear = _clipped_vector(added_mass_scale_linear, 0.0, 5.0)
    scale_angular = _clipped_vector(added_mass_scale_angular, 0.0, 5.0)

    added_linear = _added_mass_linear(rho, volume, depol, scale_linear)
    added_angular = _added_mass_angular(semi, added_linear, scale_angular)
    quadratic_linear, quadratic_angular = _quadratic_damping(semi, rho, areas, cd_linear, cd_angular)
    linear_linear, linear_angular = _linear_damping(
        quadratic_linear,
        quadratic_angular,
        reference_speed_linear,
        reference_speed_angular,
        linear_damping_ratio_linear,
        linear_damping_ratio_angular,
    )

    return EllipsoidHydroEstimate(
        semi_axes=semi,
        displaced_volume=float(volume),
        added_mass_diag=np.concatenate((added_linear, added_angular)).astype(np.float64, copy=False),
        linear_damping_diag=np.concatenate((linear_linear, linear_angular)).astype(np.float64, copy=False),
        quadratic_damping_diag=np.concatenate((quadratic_linear, quadratic_angular)).astype(np.float64, copy=False),
    )


def _clipped_vector(values: Iterable[float], lower: float, upper: float) -> np.ndarray:
    return np.clip(np.asarray(list(values), dtype=np.float64).reshape(3), lower, upper)


def _added_mass_linear(rho: float, volume: float, depol: np.ndarray, scale_linear: np.ndarray) -> np.ndarray:
    base_added_linear = rho * volume * depol / np.maximum(1.0 - depol, 1e-6)
    return base_added_linear * scale_linear


def _added_mass_angular(semi: np.ndarray, added_mass_linear: np.ndarray, scale_angular: np.ndarray) -> np.ndarray:
    radius_sq = np.array(
        [
            0.5 * (semi[1] ** 2 + semi[2] ** 2),
            0.5 * (semi[0] ** 2 + semi[2] ** 2),
            0.5 * (semi[0] ** 2 + semi[1] ** 2),
        ],
        dtype=np.float64,
    )
    paired_added_linear = np.array(
        [
            0.5 * (added_mass_linear[1] + added_mass_linear[2]),
            0.5 * (added_mass_linear[0] + added_mass_linear[2]),
            0.5 * (added_mass_linear[0] + added_mass_linear[1]),
        ],
        dtype=np.float64,
    )
    return paired_added_linear * radius_sq * scale_angular


def _quadratic_damping(
    semi: np.ndarray,
    rho: float,
    areas: np.ndarray,
    cd_linear: np.ndarray,
    cd_angular: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    quadratic_linear = 0.5 * rho * areas * cd_linear
    char_length = np.array(
        [
            0.5 * (semi[1] + semi[2]),
            0.5 * (semi[0] + semi[2]),
            0.5 * (semi[0] + semi[1]),
        ],
        dtype=np.float64,
    )
    quadratic_angular = 0.5 * rho * areas * cd_angular * (char_length ** 3)
    return quadratic_linear, quadratic_angular


def _linear_damping(
    quadratic_linear: np.ndarray,
    quadratic_angular: np.ndarray,
    reference_speed_linear: float,
    reference_speed_angular: float,
    linear_damping_ratio_linear: float,
    linear_damping_ratio_angular: float,
) -> tuple[np.ndarray, np.ndarray]:
    linear_linear = quadratic_linear * max(float(reference_speed_linear), 1e-6) * max(
        float(linear_damping_ratio_linear),
        0.0,
    )
    linear_angular = quadratic_angular * max(float(reference_speed_angular), 1e-6) * max(
        float(linear_damping_ratio_angular),
        0.0,
    )
    return linear_linear, linear_angular


__all__ = ["estimate_ellipsoid_hydrodynamics"]
