"""MuJoCo model bootstrap helpers for the main runtime."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from sim.physics.model_setup import (
    apply_fluid_geom_runtime_scales,
    apply_fluid_option_scales,
    apply_pool_runtime_overrides,
)
from sim.physics.dynamic_fluidcoef import build_dynamic_fluidcoef_setup
from sim.runtime.base_state import MuJoCoBaseState


@dataclass(frozen=True)
class ModelRuntimeSetup:
    model: Any
    data: Any
    scene_fluid_density: float
    scene_fluid_viscosity: float
    fluid_geom_ids: list[int]
    fluid_geom_names: dict[int, str]
    fluidcoef_static_geom_scales: Any
    fluidcoef_dynamic_setup: Any
    base_state: MuJoCoBaseState


def load_model_runtime_setup(
    *,
    args,
    mujoco_module,
    sim_profile: dict,
    run_mode: str,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
    env_get: Callable[[str], str | None],
    to_float_array,
    to_float_matrix,
) -> ModelRuntimeSetup:
    """Load MJCF, apply runtime fluid overrides, and derive base-state helpers."""

    model = mujoco_module.MjModel.from_xml_path(args.scene)
    data = mujoco_module.MjData(model)
    scene_fluid_density, scene_fluid_viscosity = apply_fluid_option_scales(
        model,
        sim_profile,
        env_float=env_float,
    )
    apply_pool_runtime_overrides(
        model,
        mujoco_module,
        env_float=env_float,
        run_mode=run_mode,
    )

    fluid_geom_ids, fluid_geom_names, fluidcoef_static_geom_scales = apply_fluid_geom_runtime_scales(
        model,
        mujoco_module,
        sim_profile,
        to_float_array=to_float_array,
        env_get=env_get,
    )
    fluidcoef_dynamic_setup = build_dynamic_fluidcoef_setup(
        model=model,
        sim_profile=sim_profile,
        fluid_model=str(args.fluid_model),
        fluid_geom_ids=fluid_geom_ids,
        fluid_geom_names=fluid_geom_names,
        fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
        env_flag=env_flag,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    # Initialize derived state once before runtime loops so launch start poses
    # and sensor readings use valid base position.
    mujoco_module.mj_forward(model, data)

    base_state = MuJoCoBaseState.create(
        mujoco=mujoco_module,
        model=model,
        data=data,
        water_surface_z=float(env_float("UUV_WATER_SURFACE_Z", 0.0)),
    )

    return ModelRuntimeSetup(
        model=model,
        data=data,
        scene_fluid_density=float(scene_fluid_density),
        scene_fluid_viscosity=float(scene_fluid_viscosity),
        fluid_geom_ids=list(fluid_geom_ids),
        fluid_geom_names=dict(fluid_geom_names),
        fluidcoef_static_geom_scales=fluidcoef_static_geom_scales,
        fluidcoef_dynamic_setup=fluidcoef_dynamic_setup,
        base_state=base_state,
    )


__all__ = ["ModelRuntimeSetup", "load_model_runtime_setup"]
