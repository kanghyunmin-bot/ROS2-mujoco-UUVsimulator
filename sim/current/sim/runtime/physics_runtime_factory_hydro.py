"""Hydrostatic, hydrodynamic, and underwater wrench factory steps."""

from __future__ import annotations

from sim.runtime.hydrodynamics_runtime_setup import build_hydrodynamics_runtime_setup
from sim.runtime.physics_runtime_factory_context import PhysicsRuntimeFactoryContext
from sim.runtime.physics_runtime_geometry import apply_thruster_geometry_overrides
from sim.runtime.physics_runtime_hydrostatic import build_hydrostatic_context
from sim.runtime.physics_runtime_underwater import create_underwater_wrench_runtime


def create_runtime_hydrostatic_context(ctx: PhysicsRuntimeFactoryContext):
    return build_hydrostatic_context(
        mujoco_module=ctx.mujoco_module,
        np_module=ctx.np_module,
        model=ctx.model,
        data=ctx.data,
        sim_profile=ctx.sim_profile,
        perf_cfg=ctx.perf_cfg,
        scene_fluid_density=ctx.scene_fluid_density,
        base_id=ctx.base_id,
        world_qpos_adr=ctx.world_qpos_adr,
        world_qvel_adr=ctx.world_qvel_adr,
        real_start_required=ctx.real_start_required,
        actuator_ids=ctx.actuator_ids,
        horizontal_order=ctx.horizontal_order,
        env_float=ctx.env_float,
        env_flag=ctx.env_flag,
        log=ctx.log,
    )


def apply_runtime_thruster_geometry(ctx: PhysicsRuntimeFactoryContext) -> None:
    apply_thruster_geometry_overrides(
        mujoco_module=ctx.mujoco_module,
        model=ctx.model,
        sim_profile=ctx.sim_profile,
        actuator_ids=ctx.actuator_ids,
        vertical_thrusters=ctx.vertical_thrusters,
        horizontal_thrusters=ctx.horizontal_thrusters,
        env_float=ctx.env_float,
        log=ctx.log,
    )


def create_hydrodynamics_runtime(ctx: PhysicsRuntimeFactoryContext, hydrostatic_context):
    return build_hydrodynamics_runtime_setup(
        args=ctx.args,
        model=ctx.model,
        data=ctx.data,
        mujoco_module=ctx.mujoco_module,
        sim_profile=ctx.sim_profile,
        hydro_cfg=hydrostatic_context.hydro_cfg,
        use_custom_hydrodynamics=ctx.use_custom_hydrodynamics,
        active_body_components=hydrostatic_context.hydrostatic_runtime.active_body_components,
        active_buoyancy_points=hydrostatic_context.hydrostatic_runtime.active_buoyancy_points,
        fluidcoef_dynamic_setup=ctx.fluidcoef_dynamic_setup,
        fluid_geom_names=ctx.fluid_geom_names,
        neutral_volume=hydrostatic_context.neutral_volume,
        vehicle_mass=hydrostatic_context.vehicle_mass,
        rho=hydrostatic_context.rho,
        water_surface_z=ctx.water_surface_z,
        thruster_air_force_scale=ctx.thruster_air_force_scale,
        thruster_immersion_half_height_m=ctx.thruster_immersion_half_height_m,
        env_float=ctx.env_float,
        env_flag=ctx.env_flag,
        to_float_array=ctx.to_float_array,
        log=ctx.log,
    )


def create_underwater_runtime(
    ctx: PhysicsRuntimeFactoryContext,
    hydrostatic_context,
    hydro_runtime,
    thruster_actuator_runtime,
):
    return create_underwater_wrench_runtime(
        mujoco_module=ctx.mujoco_module,
        model=ctx.model,
        data=ctx.data,
        base_id=ctx.base_id,
        world_qpos_adr=ctx.world_qpos_adr,
        world_qvel_adr=ctx.world_qvel_adr,
        water_surface_z=ctx.water_surface_z,
        use_custom_hydrodynamics=ctx.use_custom_hydrodynamics,
        hydrostatic_context=hydrostatic_context,
        hydro_runtime=hydro_runtime,
        initial_depth_hold=ctx.initial_depth_hold,
        thruster_actuator_runtime=thruster_actuator_runtime,
        base_origin_world=ctx.base_origin_world,
    )


__all__ = [
    "create_runtime_hydrostatic_context",
    "apply_runtime_thruster_geometry",
    "create_hydrodynamics_runtime",
    "create_underwater_runtime",
]
