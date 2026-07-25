"""Compatibility exports for MuJoCo geom fluid coefficient scaling."""

from __future__ import annotations

from sim.physics.fluidcoef_scale_extra import apply_extra_fluidcoef_scale
from sim.physics.fluidcoef_scale_global import apply_global_fluidcoef_scale
from sim.physics.fluidcoef_scale_per_geom import apply_per_geom_fluidcoef_scales


__all__ = [
    "apply_extra_fluidcoef_scale",
    "apply_global_fluidcoef_scale",
    "apply_per_geom_fluidcoef_scales",
]
