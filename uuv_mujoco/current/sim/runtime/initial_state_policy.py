"""Initial hold and real-start policy resolution."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sim.runtime.initial_hold import InitialDepthHoldState
from sim.runtime.initial_state_depths import InitialBar30Depth, real_start_tolerances
from sim.runtime.initial_state_types import InitialRealStartPolicy


def create_initial_depth_hold_state(*, args: Any, initial_bar30: InitialBar30Depth) -> dict:
    return InitialDepthHoldState.create(
        active=bool(args.hold_initial_depth_until_release),
        initial_depth_m=float(args.initial_depth_m) if args.initial_depth_m is not None else None,
        bar30_depth_m=initial_bar30.value_m,
        release_linear_velocity_body=args.release_linear_velocity_body,
        release_angular_velocity_body=args.release_angular_velocity_body,
    )


def resolve_initial_real_start_policy(
    *,
    args: Any,
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> InitialRealStartPolicy:
    depth_tol_m, attitude_tol_rad, velocity_tol_mps = real_start_tolerances(env_float=env_float)
    return InitialRealStartPolicy(
        auto_release=bool(
            args.sitl
            and env_flag(
                "UUV_REAL_START_STATE_AUTO_RELEASE",
                env_flag("UUV_REAL_START_STATE", False),
            )
        ),
        required=bool(env_flag("UUV_REAL_START_STATE", False)),
        depth_tol_m=depth_tol_m,
        attitude_tol_rad=attitude_tol_rad,
        velocity_tol_mps=velocity_tol_mps,
    )


__all__ = ["create_initial_depth_hold_state", "resolve_initial_real_start_policy"]
