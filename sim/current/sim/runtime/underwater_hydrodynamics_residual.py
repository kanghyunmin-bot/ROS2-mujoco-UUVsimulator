"""Compatibility facade for residual hydrodynamic wrench application."""

from __future__ import annotations

from sim.runtime.underwater_cfd_apply import apply_cfd_dynamic_wrench
from sim.runtime.underwater_fossen_apply import (
    apply_fossen_added_mass_wrench,
    apply_fossen_residual_wrench,
)
from sim.runtime.underwater_residual_apply import apply_residual_hydro


__all__ = [
    "apply_residual_hydro",
    "apply_fossen_added_mass_wrench",
    "apply_fossen_residual_wrench",
    "apply_cfd_dynamic_wrench",
]
