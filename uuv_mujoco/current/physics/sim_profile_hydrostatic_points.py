"""Compatibility surface for body-component and buoyancy-point parsers."""

from __future__ import annotations

from .sim_profile_hydrostatic_body_components import parse_body_components
from .sim_profile_hydrostatic_buoyancy_points import parse_buoyancy_points
from .sim_profile_hydrostatic_normalize import (
    normalize_buoyancy_model,
    normalize_component_shape,
    normalize_hydrostatic_volume_source,
)

__all__ = [
    "normalize_buoyancy_model",
    "normalize_component_shape",
    "normalize_hydrostatic_volume_source",
    "parse_body_components",
    "parse_buoyancy_points",
]
