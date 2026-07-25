"""Underwater wrench runtime builder."""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from sim.runtime.physics_step_callbacks import body_velocity_local_factory
from sim.runtime.underwater_wrench_runtime import UnderwaterWrenchRuntime


def create_underwater_wrench_runtime(
    *,
    mujoco_module: Any,
    model: Any,
    data: Any,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
    water_surface_z: float,
    use_custom_hydrodynamics: bool,
    hydrostatic_context: Any,
    hydro_runtime: Any,
    initial_depth_hold: dict[str, Any],
    thruster_actuator_runtime: Any,
    base_origin_world: Callable[[], np.ndarray],
) -> tuple[UnderwaterWrenchRuntime, Callable[[], np.ndarray]]:
    body_velocity_local = body_velocity_local_factory(
        mujoco_module=mujoco_module,
        model=model,
        data=data,
        base_id=base_id,
    )
    underwater_wrench_runtime = UnderwaterWrenchRuntime(
        model=model,
        data=data,
        base_id=base_id,
        world_qpos_adr=world_qpos_adr,
        world_qvel_adr=world_qvel_adr,
        water_surface_z=water_surface_z,
        rho=hydrostatic_context.rho,
        gravity=hydrostatic_context.gravity,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        hydrostatic=hydrostatic_context.hydrostatic_runtime,
        hydrodynamics=hydro_runtime,
        initial_depth_hold=initial_depth_hold,
        thruster_actuator_runtime=thruster_actuator_runtime,
        base_origin_world=base_origin_world,
        body_velocity_local=body_velocity_local,
        update_dynamic_fluidcoef=hydro_runtime.update_dynamic_fluidcoef,
    )
    return underwater_wrench_runtime, body_velocity_local


__all__ = ["create_underwater_wrench_runtime"]
