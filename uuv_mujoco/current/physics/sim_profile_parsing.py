"""Compatibility exports for simulation profile parsers."""

from __future__ import annotations

from .sim_profile_hydrostatic_points import (
    normalize_buoyancy_model,
    normalize_component_shape,
    normalize_hydrostatic_volume_source,
    parse_body_components,
    parse_buoyancy_points,
)
from .sim_profile_hydrostatic_restoring import parse_hydrostatic_restoring
from .sim_profile_parse_common import (
    bool_from_value,
    clamp,
    to_float_array,
    vector3_from_keys,
    vector_from_keys,
)


__all__ = [
    "bool_from_value",
    "clamp",
    "normalize_buoyancy_model",
    "normalize_component_shape",
    "normalize_hydrostatic_volume_source",
    "parse_body_components",
    "parse_buoyancy_points",
    "parse_hydrostatic_restoring",
    "to_float_array",
    "vector3_from_keys",
    "vector_from_keys",
]
