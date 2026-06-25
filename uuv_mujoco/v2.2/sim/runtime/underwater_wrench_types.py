"""Typed intermediate results for underwater wrench runtime."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HydrostaticWrenchResult:
    submerged: float
    buoyancy_submerged: float
    buoy_force_world: np.ndarray
    buoy_tau_world: np.ndarray
    buoy_point_world: np.ndarray
