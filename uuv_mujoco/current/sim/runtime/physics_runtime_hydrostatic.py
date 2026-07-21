"""Hydrostatic setup facade for runtime physics wiring."""

from __future__ import annotations

from typing import Any, Callable

from sim.physics.body_distribution import apply_body_component_distribution
from sim.physics.horizontal_allocator import HorizontalAllocator
from sim.physics.hydrostatic_setup import configure_hydrostatic_runtime
from sim.runtime.physics_runtime_hydro_config import (
    build_runtime_hydrodynamics_config,
    performance_force_max,
)
from sim.runtime.physics_runtime_mass_reference import build_mass_reference
from sim.runtime.physics_runtime_types import HydrostaticContext


def build_hydrostatic_context(
    *,
    mujoco_module: Any,
    np_module: Any,
    model: Any,
    data: Any,
    sim_profile: dict[str, Any],
    perf_cfg: dict[str, Any],
    scene_fluid_density: float,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
    real_start_required: bool,
    actuator_ids: dict[str, int],
    horizontal_order: list[str],
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    log: Callable[[str], None],
) -> HydrostaticContext:
    hydro_cfg = build_runtime_hydrodynamics_config(
        np_module=np_module,
        sim_profile=sim_profile,
        perf_cfg=perf_cfg,
        scene_fluid_density=scene_fluid_density,
    )
    apply_body_component_distribution(
        mujoco=mujoco_module,
        model=model,
        data=data,
        components=hydro_cfg.body_components,
        sim_profile=sim_profile,
        base_id=base_id,
        world_qpos_adr=world_qpos_adr,
        world_qvel_adr=world_qvel_adr,
    )
    hydrostatic_runtime = configure_hydrostatic_runtime(
        model=model,
        mujoco_module=mujoco_module,
        base_id=base_id,
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
        real_start_required=real_start_required,
        env_float=env_float,
        env_flag=env_flag,
        log=log,
    )
    horizontal_allocator = HorizontalAllocator.from_model(
        model=model,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        base_id=base_id,
        order=horizontal_order,
    )

    mass_ref = build_mass_reference(
        np_module=np_module,
        model=model,
        base_id=base_id,
        scene_fluid_density=scene_fluid_density,
        log=log,
    )
    return HydrostaticContext(
        hydro_cfg=hydro_cfg,
        hydrostatic_runtime=hydrostatic_runtime,
        horizontal_allocator=horizontal_allocator,
        rho=mass_ref.rho,
        gravity=mass_ref.gravity,
        vehicle_mass=mass_ref.vehicle_mass,
        neutral_volume=mass_ref.neutral_volume,
    )


__all__ = ["build_hydrostatic_context", "performance_force_max"]
