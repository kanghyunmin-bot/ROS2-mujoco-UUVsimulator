"""Compatibility exports for underwater vehicle dynamics helpers."""

from __future__ import annotations

from .ellipsoid_hydrodynamics import (
    EllipsoidHydroEstimate,
    ellipsoid_depolarization_factors,
    ellipsoid_projected_areas,
    ellipsoid_volume,
    estimate_ellipsoid_hydrodynamics,
)
from .hydrodynamics_math import added_mass_coriolis, first_order_response, skew
from .hydrostatic_fraction_helpers import (
    submerged_fraction,
    submerged_fraction_ellipsoid,
    submerged_fraction_linear,
)
from .thruster_curve_helpers import (
    polyval_ascending,
    scaled_polynomial_force,
    shape_thruster_command,
)

__all__ = [
    "EllipsoidHydroEstimate",
    "added_mass_coriolis",
    "ellipsoid_depolarization_factors",
    "ellipsoid_projected_areas",
    "ellipsoid_volume",
    "estimate_ellipsoid_hydrodynamics",
    "first_order_response",
    "polyval_ascending",
    "scaled_polynomial_force",
    "shape_thruster_command",
    "skew",
    "submerged_fraction",
    "submerged_fraction_ellipsoid",
    "submerged_fraction_linear",
]
