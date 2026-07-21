"""Compatibility exports for MuJoCo fluid geom/coef runtime scaling."""

from __future__ import annotations

from sim.physics.fluid_geom_apply import apply_fluid_geom_runtime_scales
from sim.physics.fluid_geom_common import (
    ArrayParser,
    fluid_geom_mask,
    fluid_geom_name,
    matching_geom_ids,
    parse_geom_size_scale,
    parse_space_separated_floats,
)
from sim.physics.fluid_geom_size_runtime import apply_extra_geom_size_scale, apply_profile_geom_size_scales
from sim.physics.fluidcoef_scale_runtime import (
    apply_extra_fluidcoef_scale,
    apply_global_fluidcoef_scale,
    apply_per_geom_fluidcoef_scales,
)


_fluid_geom_name = fluid_geom_name
_parse_geom_size_scale = parse_geom_size_scale
_fluid_geom_mask = fluid_geom_mask
_matching_geom_ids = matching_geom_ids
_parse_space_separated_floats = parse_space_separated_floats
_apply_profile_geom_size_scales = apply_profile_geom_size_scales
_apply_extra_geom_size_scale = apply_extra_geom_size_scale
_apply_global_fluidcoef_scale = apply_global_fluidcoef_scale
_apply_per_geom_fluidcoef_scales = apply_per_geom_fluidcoef_scales
_apply_extra_fluidcoef_scale = apply_extra_fluidcoef_scale


__all__ = [
    "ArrayParser",
    "_apply_extra_fluidcoef_scale",
    "_apply_extra_geom_size_scale",
    "_apply_global_fluidcoef_scale",
    "_apply_per_geom_fluidcoef_scales",
    "_apply_profile_geom_size_scales",
    "_fluid_geom_mask",
    "_fluid_geom_name",
    "_matching_geom_ids",
    "_parse_geom_size_scale",
    "_parse_space_separated_floats",
    "apply_fluid_geom_runtime_scales",
]
