"""Equivalent-ellipsoid baseline hydrodynamic coefficient estimates."""

from __future__ import annotations

from .ellipsoid_geometry import (
    ellipsoid_depolarization_factors,
    ellipsoid_projected_areas,
    ellipsoid_volume,
)
from .ellipsoid_hydro_coefficients import estimate_ellipsoid_hydrodynamics
from .ellipsoid_hydro_types import EllipsoidHydroEstimate

__all__ = [
    "EllipsoidHydroEstimate",
    "ellipsoid_depolarization_factors",
    "ellipsoid_projected_areas",
    "ellipsoid_volume",
    "estimate_ellipsoid_hydrodynamics",
]
