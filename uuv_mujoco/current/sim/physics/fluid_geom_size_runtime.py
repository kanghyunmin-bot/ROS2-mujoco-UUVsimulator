"""Runtime scaling facade for MuJoCo fluid geom sizes."""

from __future__ import annotations

from sim.physics.fluid_geom_size_extra import apply_extra_geom_size_scale
from sim.physics.fluid_geom_size_profile import apply_profile_geom_size_scales


__all__ = ["apply_extra_geom_size_scale", "apply_profile_geom_size_scales"]
