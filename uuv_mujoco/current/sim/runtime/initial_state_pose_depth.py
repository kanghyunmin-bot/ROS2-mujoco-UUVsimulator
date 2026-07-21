"""Initial Bar30/base-depth application helpers."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from sim.runtime.initial_state_depth_apply import apply_initial_bar30_depth, apply_initial_base_depth
from sim.runtime.initial_state_depths import InitialBar30Depth


def apply_initial_depth_request(
    *,
    args: Any,
    base_state: Any,
    initial_depth_hold: Any,
    initial_bar30: InitialBar30Depth,
    env_float: Callable[[str, float], float],
) -> None:
    if initial_bar30.value_m is not None:
        apply_initial_bar30_depth(
            args=args,
            base_state=base_state,
            initial_depth_hold=initial_depth_hold,
            initial_bar30=initial_bar30,
            env_float=env_float,
        )
        return
    if args.initial_depth_m is not None:
        apply_initial_base_depth(
            args=args,
            base_state=base_state,
            initial_depth_hold=initial_depth_hold,
            env_float=env_float,
        )


__all__ = ["apply_initial_depth_request"]
