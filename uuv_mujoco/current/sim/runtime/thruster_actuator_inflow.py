"""Runtime adapter for local axial thruster inflow."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.physics.thruster_inflow import ThrusterInflowConfig, apply_thruster_inflow


def force_with_local_inflow(
    runtime: Any,
    *,
    name: str,
    aid: int,
    sid: int,
    base_id: int,
    base_rot: np.ndarray,
    static_force_n: float,
    command_fraction: float,
    config: ThrusterInflowConfig,
) -> float:
    """Return static thrust corrected by the water velocity at its site."""

    if not config.enabled or abs(static_force_n) <= 1.0e-12:
        runtime.last_inflow_multiplier[name] = 1.0
        runtime.last_axial_advance_speed_mps[name] = 0.0
        return float(static_force_n)

    position_world = _object_position_world(runtime, sid=sid, base_id=base_id)
    site_velocity_world = _object_velocity_world(runtime, sid=sid, base_id=base_id)
    water_velocity_world = _water_velocity_world(runtime, position_world)
    thrust_axis_local = np.asarray(
        runtime.model.actuator_gear[aid, :3],
        dtype=np.float64,
    )
    thrust_axis_world = (
        np.asarray(runtime.data.site_xmat[sid], dtype=np.float64).reshape(3, 3)
        @ thrust_axis_local
        if sid >= 0
        else base_rot @ thrust_axis_local
    )
    result = apply_thruster_inflow(
        static_force_n,
        command_fraction=command_fraction,
        thrust_axis_world=thrust_axis_world,
        site_velocity_world_mps=site_velocity_world,
        water_velocity_world_mps=water_velocity_world,
        config=config,
    )
    runtime.last_inflow_multiplier[name] = result.multiplier
    runtime.last_axial_advance_speed_mps[name] = result.axial_advance_speed_mps
    return float(result.force_n)


def _object_position_world(runtime: Any, *, sid: int, base_id: int) -> np.ndarray:
    if sid >= 0:
        return np.asarray(runtime.data.site_xpos[sid], dtype=np.float64).copy()
    return np.asarray(runtime.data.xpos[base_id], dtype=np.float64).copy()


def _object_velocity_world(runtime: Any, *, sid: int, base_id: int) -> np.ndarray:
    velocity = np.zeros(6, dtype=np.float64)
    object_type = (
        runtime.mujoco_module.mjtObj.mjOBJ_SITE
        if sid >= 0
        else runtime.mujoco_module.mjtObj.mjOBJ_BODY
    )
    object_id = sid if sid >= 0 else base_id
    runtime.mujoco_module.mj_objectVelocity(
        runtime.model,
        runtime.data,
        object_type,
        int(object_id),
        velocity,
        0,
    )
    # MuJoCo spatial vectors store angular velocity first, linear velocity last.
    return velocity[3:6]


def _water_velocity_world(runtime: Any, position_world: np.ndarray) -> np.ndarray:
    sampler = runtime.current_velocity_sampler
    if sampler is not None:
        sample = sampler(position_world.copy(), float(runtime.data.time))
    else:
        sample = runtime.model.opt.wind
    velocity = np.asarray(sample, dtype=np.float64)
    if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
        raise ValueError("thruster current sampler must return three finite values")
    return velocity


__all__ = ["force_with_local_inflow"]
