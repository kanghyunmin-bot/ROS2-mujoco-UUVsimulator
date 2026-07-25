"""Compatibility facade for dynamic MuJoCo fluidcoef helpers."""

from __future__ import annotations

from sim.physics.dynamic_fluidcoef_loads import (
    axis_weighted_load,
    fluidcoef_loads_from_local_velocity,
)
from sim.physics.dynamic_fluidcoef_runtime import DynamicFluidcoefRuntime
from sim.physics.dynamic_fluidcoef_setup import build_dynamic_fluidcoef_setup
from sim.physics.dynamic_fluidcoef_types import (
    DEFAULT_FLUIDCOEF_ANGULAR_AXIS_WEIGHTS,
    DEFAULT_FLUIDCOEF_AXIS_WEIGHTS,
    DynamicFluidcoefSetup,
)

__all__ = [
    "DEFAULT_FLUIDCOEF_ANGULAR_AXIS_WEIGHTS",
    "DEFAULT_FLUIDCOEF_AXIS_WEIGHTS",
    "DynamicFluidcoefRuntime",
    "DynamicFluidcoefSetup",
    "axis_weighted_load",
    "build_dynamic_fluidcoef_setup",
    "fluidcoef_loads_from_local_velocity",
]
