"""MuJoCo model bootstrap helpers for the main runtime."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable

from sim.physics.model_setup import (
    apply_fluid_geom_runtime_scales,
    apply_fluid_option_scales,
    apply_pool_runtime_overrides,
)
from sim.physics.dynamic_fluidcoef import build_dynamic_fluidcoef_setup
from sim.runtime.base_state import MuJoCoBaseState
from sim.runtime.model_binary_cache import load_model_with_binary_cache


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

    model_cache_dir = env_get("UUV_MUJOCO_MODEL_CACHE_DIR")
    model = load_model_with_binary_cache(
        mujoco_module=mujoco_module,
        scene=args.scene,
        enabled=env_flag("UUV_MUJOCO_MODEL_CACHE", True),
        cache_dir=model_cache_dir,
        log=lambda message: print(message, flush=True),
    )
    _apply_render_quality_override(model, env_float=env_float)
    _apply_timestep_override(
        model,
        mujoco_module=mujoco_module,
        env_float=env_float,
        env_flag=env_flag,
    )
    if bool(getattr(args, "sitl", False)):
        loop_hz = float(env_float("SITL_SCHED_LOOP_RATE", 400.0))
        if not math.isfinite(loop_hz) or loop_hz <= 0.0:
            raise ValueError("SITL_SCHED_LOOP_RATE must be positive and finite")
        _align_fcu_timestep(model, loop_hz)
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


def _apply_render_quality_override(model: Any, *, env_float: Callable[[str, float], float]) -> None:
    quality = getattr(getattr(model, "vis", None), "quality", None)
    if quality is None:
        return
    old_shadow = int(quality.shadowsize)
    old_samples = int(quality.offsamples)
    shadow = int(max(256, min(8192, env_float("UUV_MUJOCO_SHADOW_SIZE", old_shadow))))
    samples = int(max(1, min(8, env_float("UUV_MUJOCO_OFFSAMPLES", old_samples))))
    quality.shadowsize = shadow
    quality.offsamples = samples
    if shadow != old_shadow or samples != old_samples:
        print(
            "[runtime] render quality override: "
            f"shadow={old_shadow}->{shadow} offsamples={old_samples}->{samples}",
            flush=True,
        )


def _apply_timestep_override(
    model: Any,
    *,
    mujoco_module: Any,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> None:
    current_timestep = float(model.opt.timestep)
    requested_timestep = float(env_float("UUV_MUJOCO_TIMESTEP", current_timestep))
    if requested_timestep <= 0.0:
        return
    # Thin CAD rake fingers must resolve contact before a light buoy jig can
    # cross them. Scene-local limits also apply to larger GUI/env step requests.
    if int(getattr(model, "nnumeric", 0)):
        limit_id = mujoco_module.mj_name2id(
            model, mujoco_module.mjtObj.mjOBJ_NUMERIC, "buoy_contact_max_timestep"
        )
        if limit_id >= 0:
            limit = float(model.numeric_data[model.numeric_adr[limit_id]])
            if limit > 0 and requested_timestep > limit:
                print(f"[runtime] CAD buoy contact timestep: {requested_timestep:.6f}s capped={limit:.6f}s", flush=True)
                requested_timestep = limit
    guard_course_buoy_contacts = env_flag("UUV_COURSE_BUOY_TIMESTEP_GUARD", True)
    if guard_course_buoy_contacts and requested_timestep > 0.005 and _model_has_course_buoys(
        model,
        mujoco_module=mujoco_module,
    ):
        print(
            "[runtime] course buoy contact timestep guard: "
            f"requested={requested_timestep:.4f}s capped=0.0050s",
            flush=True,
        )
        requested_timestep = 0.005
    bounded_timestep = max(0.001, min(0.030, requested_timestep))
    if abs(bounded_timestep - current_timestep) <= 1.0e-12:
        return
    model.opt.timestep = bounded_timestep
    print(
        "[runtime] MuJoCo timestep override: "
        f"{current_timestep:.4f}s -> {bounded_timestep:.4f}s",
        flush=True,
    )


def _model_has_course_buoys(model: Any, *, mujoco_module: Any) -> bool:
    obj_body = mujoco_module.mjtObj.mjOBJ_BODY
    for body_id in range(int(model.nbody)):
        body_name = mujoco_module.mj_id2name(model, obj_body, body_id) or ""
        if body_name.startswith("course_buoy_"):
            return True
    return False


__all__ = ["ModelRuntimeSetup", "load_model_runtime_setup"]


def _align_fcu_timestep(model, loop_hz: float) -> None:
    """Align physics to integer subdivisions of the FCU period."""
    period = 1.0 / loop_hz
    step = period / max(1, math.ceil(period / float(model.opt.timestep) - 1e-9))
    if abs(float(model.opt.timestep) - step) > 1e-12:
        print(f"[runtime] FCU clock aligned physics timestep: {model.opt.timestep:.6f}s -> {step:.6f}s", flush=True)
    model.opt.timestep = step
