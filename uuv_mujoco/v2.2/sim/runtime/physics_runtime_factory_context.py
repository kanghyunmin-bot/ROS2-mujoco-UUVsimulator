"""Context record for runtime physics factory wiring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class PhysicsRuntimeFactoryContext:
    args: Any
    mujoco_module: Any
    np_module: Any
    model: Any
    data: Any
    sim_profile: dict[str, Any]
    perf_cfg: dict[str, Any]
    scene_fluid_density: float
    base_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    fluidcoef_dynamic_setup: Any
    fluid_geom_names: dict[int, str]
    water_surface_z: float
    base_origin_world: Callable[[], np.ndarray]
    real_start_required: bool
    use_custom_hydrodynamics: bool
    actuator_ids: dict[str, int]
    ctrlrange: dict[str, tuple[float, float]]
    thruster_params_path: Any
    all_thruster_names: list[str]
    horizontal_order: list[str]
    vertical_names: list[str]
    vertical_thrusters: list[str]
    horizontal_thrusters: list[str]
    servo_map: list[int]
    servo_signs: list[float]
    plant_replay_direct_rcout: bool
    ros_bridge_runtime: Any
    initial_depth_hold: dict[str, Any]
    command_state: Any
    thruster_air_force_scale: float
    thruster_immersion_half_height_m: float
    env_float: Callable[[str, float], float]
    env_flag: Callable[[str, bool], bool]
    to_float_array: Callable[[str, np.ndarray], np.ndarray]
    log: Callable[[str], None]


__all__ = ["PhysicsRuntimeFactoryContext"]
