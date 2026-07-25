"""Sim-profile-derived candidate facade for automatic initial Bar30 depth."""

from __future__ import annotations

from .initial_depth_body_components import body_component_depth_candidates
from .initial_depth_buoyancy_points import buoyancy_point_depth_candidates


__all__ = [
    "body_component_depth_candidates",
    "buoyancy_point_depth_candidates",
]
