"""Types for equivalent-ellipsoid hydrodynamic estimates."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EllipsoidHydroEstimate:
    """Baseline 6DOF coefficients derived from an equivalent ellipsoid."""

    semi_axes: np.ndarray
    displaced_volume: float
    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray


__all__ = ["EllipsoidHydroEstimate"]
