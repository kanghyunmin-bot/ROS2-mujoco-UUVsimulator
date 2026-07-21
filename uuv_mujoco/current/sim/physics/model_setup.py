"""Compatibility facade for model-level MuJoCo runtime setup helpers."""

from __future__ import annotations

from sim.physics.body_tree import body_subtree_mass, build_body_children
from sim.physics.fluid_geom_runtime import apply_fluid_geom_runtime_scales
from sim.physics.fluid_option_runtime import apply_fluid_option_scales
from sim.physics.pool_runtime_overrides import apply_pool_runtime_overrides

__all__ = [
    "apply_fluid_geom_runtime_scales",
    "apply_fluid_option_scales",
    "apply_pool_runtime_overrides",
    "body_subtree_mass",
    "build_body_children",
]
