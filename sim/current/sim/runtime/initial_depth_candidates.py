"""Compatibility exports for automatic initial Bar30 depth candidate builders."""

from __future__ import annotations

from .initial_depth_geometry import (
    finite_depth_candidates,
    required_bar30_depth_for_top,
    select_auto_initial_depth,
    world_z_from_base_local,
)
from .initial_depth_model_candidates import (
    fluid_geom_depth_candidates,
    thruster_depth_candidates,
)
from .initial_depth_profile_candidates import (
    body_component_depth_candidates,
    buoyancy_point_depth_candidates,
)


__all__ = [
    "body_component_depth_candidates",
    "buoyancy_point_depth_candidates",
    "finite_depth_candidates",
    "fluid_geom_depth_candidates",
    "required_bar30_depth_for_top",
    "select_auto_initial_depth",
    "thruster_depth_candidates",
    "world_z_from_base_local",
]
