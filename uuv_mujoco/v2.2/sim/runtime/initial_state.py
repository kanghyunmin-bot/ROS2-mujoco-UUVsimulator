"""Initial pose/depth setup for the MuJoCo runner."""

from __future__ import annotations

from collections.abc import Callable, Sequence

from sim.runtime.initial_state_application import apply_initial_runtime_requests
from sim.runtime.initial_state_depths import (
    apply_drop_start_default,
    resolve_initial_bar30_depth,
)
from sim.runtime.initial_state_policy import (
    create_initial_depth_hold_state,
    resolve_initial_real_start_policy,
)
from sim.runtime.initial_state_pressure_calibration import calibrate_initial_bar30_surface_pressure
from sim.runtime.initial_state_types import InitialRuntimeState


def configure_initial_runtime_state(
    *,
    args,
    mujoco,
    model,
    data,
    sim_profile: dict,
    base_state,
    fluid_geom_ids,
    fluid_geom_names,
    thruster_names: Sequence[str],
    thruster_immersion_half_height_m: float,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> InitialRuntimeState:
    initial_bar30 = resolve_initial_bar30_depth(
        args=args,
        mujoco=mujoco,
        model=model,
        data=data,
        sim_profile=sim_profile,
        base_state=base_state,
        fluid_geom_ids=fluid_geom_ids,
        fluid_geom_names=fluid_geom_names,
        thruster_names=thruster_names,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        env_float=env_float,
    )
    apply_drop_start_default(args=args, env_float=env_float)

    initial_depth_hold = create_initial_depth_hold_state(
        args=args,
        initial_bar30=initial_bar30,
    )
    real_start_policy = resolve_initial_real_start_policy(
        args=args,
        env_float=env_float,
        env_flag=env_flag,
    )

    apply_initial_runtime_requests(
        args=args,
        data=data,
        base_state=base_state,
        initial_depth_hold=initial_depth_hold,
        initial_bar30=initial_bar30,
        env_float=env_float,
    )

    calibrate_initial_bar30_surface_pressure(
        model=model,
        base_state=base_state,
        env_flag=env_flag,
        env_float=env_float,
    )

    return InitialRuntimeState(
        initial_depth_hold=initial_depth_hold,
        initial_depth_hold_auto_release=real_start_policy.auto_release,
        real_start_required=real_start_policy.required,
        real_start_depth_tol_m=real_start_policy.depth_tol_m,
        real_start_attitude_tol_rad=real_start_policy.attitude_tol_rad,
        real_start_velocity_tol_mps=real_start_policy.velocity_tol_mps,
    )


__all__ = ["InitialRuntimeState", "configure_initial_runtime_state"]
