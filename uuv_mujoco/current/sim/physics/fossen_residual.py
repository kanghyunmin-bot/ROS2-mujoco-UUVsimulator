"""Compatibility exports for Fossen-style residual hydrodynamics."""

from __future__ import annotations

from sim.physics.fossen_residual_builders import build_fossen_residual_runtime, build_residual_hydro_runtime
from sim.physics.fossen_residual_types import (
    FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS,
    FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS,
    FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS,
    FOSSEN_RESIDUAL_LINEAR_KEYS,
    FOSSEN_RESIDUAL_QUADRATIC_KEYS,
    FossenResidualRuntime,
    RESIDUAL_HYDRO_KEYS,
    ResidualHydroRuntime,
)
from sim.physics.fossen_residual_wrench import fossen_residual_wrench_body


__all__ = [
    "FOSSEN_RESIDUAL_ADDED_MASS_COUPLING_KEYS",
    "FOSSEN_RESIDUAL_ADDED_MASS_DIAG_KEYS",
    "FOSSEN_RESIDUAL_FORWARD_SPEED_KEYS",
    "FOSSEN_RESIDUAL_LINEAR_KEYS",
    "FOSSEN_RESIDUAL_QUADRATIC_KEYS",
    "FossenResidualRuntime",
    "RESIDUAL_HYDRO_KEYS",
    "ResidualHydroRuntime",
    "build_fossen_residual_runtime",
    "build_residual_hydro_runtime",
    "fossen_residual_wrench_body",
]
