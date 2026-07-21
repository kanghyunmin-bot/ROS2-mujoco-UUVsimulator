"""Typed container for MuJoCo runner model and initial-state setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sim.runtime.initial_depth_runtime import InitialDepthHoldRuntime
from sim.runtime.model_runtime_setup import ModelRuntimeSetup


@dataclass
class RunnerInitialSetup:
    model_setup: ModelRuntimeSetup
    model: Any
    data: Any
    scene_fluid_density: float
    scene_fluid_viscosity: float
    fluid_geom_ids: list[int]
    fluid_geom_names: dict[int, str]
    fluidcoef_dynamic_setup: Any
    base_state: Any
    base_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    water_surface_z: float
    base_origin_world: Any
    set_base_depth: Any
    bar30_depth_now_m: Any
    set_bar30_depth: Any
    thruster_air_force_scale: float
    thruster_immersion_half_height_m: float
    initial_runtime_state: Any
    initial_depth_hold: dict
    initial_depth_hold_auto_release: bool
    real_start_required: bool
    initial_depth_runtime: InitialDepthHoldRuntime
    apply_initial_depth_hold: Any
    apply_release_velocity_state: Any


__all__ = ["RunnerInitialSetup"]
