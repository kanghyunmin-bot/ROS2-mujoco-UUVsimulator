"""Apply initial depth, pose, and hold capture in order."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sim.runtime.initial_state_depths import InitialBar30Depth
from sim.runtime.initial_state_pose import (
    apply_initial_depth_request,
    apply_initial_position_and_attitude,
    capture_configured_initial_depth_hold,
)


def apply_initial_runtime_requests(
    *,
    args: Any,
    data: Any,
    base_state: Any,
    initial_depth_hold: dict,
    initial_bar30: InitialBar30Depth,
    env_float: Callable[[str, float], float],
) -> None:
    apply_initial_depth_request(
        args=args,
        base_state=base_state,
        initial_depth_hold=initial_depth_hold,
        initial_bar30=initial_bar30,
        env_float=env_float,
    )
    apply_initial_position_and_attitude(
        args=args,
        base_state=base_state,
        initial_bar30=initial_bar30,
    )
    capture_configured_initial_depth_hold(
        args=args,
        initial_depth_hold=initial_depth_hold,
        data=data,
        base_state=base_state,
        initial_bar30=initial_bar30,
    )


__all__ = ["apply_initial_runtime_requests"]
