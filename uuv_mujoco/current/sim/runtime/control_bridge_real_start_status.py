"""Real-start status construction for the runtime control bridge."""

from __future__ import annotations

from .pose_math import rpy_rad_from_quat_wxyz
from .real_start_runtime import RealStartRuntimeStatus


def create_runtime_real_start_status(
    *,
    ros_bridge_runtime,
    env_float,
    initial_runtime_state,
    initial_depth_hold: dict,
    water_surface_z: float,
    scene_fluid_density: float,
    base_origin_world,
    bar30_depth_now_m,
    data,
    model,
    world_qpos_adr: int,
) -> RealStartRuntimeStatus:
    return RealStartRuntimeStatus.create(
        ros_bridge=ros_bridge_runtime.get(),
        env_float=env_float,
        required=initial_runtime_state.real_start_required,
        hold_active_fn=lambda: bool(initial_depth_hold["active"]),
        base_depth_m_fn=lambda: float(water_surface_z - float(base_origin_world()[2])),
        bar30_depth_m_fn=bar30_depth_now_m,
        base_xy_m_fn=lambda: data.qpos[world_qpos_adr : world_qpos_adr + 2].copy(),
        current_rpy_rad_fn=lambda: rpy_rad_from_quat_wxyz(
            data.qpos[world_qpos_adr + 3 : world_qpos_adr + 7]
        ),
        release_linear_velocity_body_fn=lambda: initial_depth_hold.get("release_linear_velocity_body"),
        release_angular_velocity_body_fn=lambda: initial_depth_hold.get("release_angular_velocity_body"),
        # The fluid ownership contract intentionally zeros model.opt.density
        # for Python-owned hydrodynamics. BAR30 parity must retain the scene's
        # immutable water density instead of observing that solver switch.
        model_density_fn=lambda: float(scene_fluid_density),
        depth_tolerance_m=initial_runtime_state.real_start_depth_tol_m,
        attitude_tolerance_rad=initial_runtime_state.real_start_attitude_tol_rad,
        velocity_tolerance_mps=initial_runtime_state.real_start_velocity_tol_mps,
    )


__all__ = ["create_runtime_real_start_status"]
