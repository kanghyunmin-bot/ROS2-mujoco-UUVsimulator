"""Compatibility exports for residual hydrodynamics runtime builders."""

from __future__ import annotations

from sim.physics.fossen_residual_runtime_builder import build_fossen_residual_runtime
from sim.physics.residual_hydro_runtime_builder import build_residual_hydro_runtime


__all__ = ["build_fossen_residual_runtime", "build_residual_hydro_runtime"]
