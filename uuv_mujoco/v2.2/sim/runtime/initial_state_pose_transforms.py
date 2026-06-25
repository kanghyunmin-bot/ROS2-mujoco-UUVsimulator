"""Initial horizontal position and attitude application helpers."""

from __future__ import annotations

from typing import Any

from sim.runtime.initial_state_depths import InitialBar30Depth
from sim.runtime.initial_state_pose_logging import log_initial_attitude, log_initial_horizontal_position
from sim.runtime.initial_state_pose_vectors import initial_rpy_vector, initial_xy_vector


def apply_initial_position_and_attitude(
    *,
    args: Any,
    base_state: Any,
    initial_bar30: InitialBar30Depth,
) -> None:
    _apply_initial_horizontal_position(
        args=args,
        base_state=base_state,
        initial_bar30=initial_bar30,
    )
    _apply_initial_attitude(
        args=args,
        base_state=base_state,
        initial_bar30=initial_bar30,
    )


def _apply_initial_horizontal_position(
    *,
    args: Any,
    base_state: Any,
    initial_bar30: InitialBar30Depth,
) -> None:
    initial_xy = initial_xy_vector(args)
    if initial_xy is None:
        return
    base_state.set_base_position_xy(float(initial_xy[0]), float(initial_xy[1]))
    if initial_bar30.value_m is not None:
        base_state.set_bar30_depth(float(initial_bar30.value_m))
    elif args.initial_depth_m is not None:
        base_state.set_base_depth(float(args.initial_depth_m))
    log_initial_horizontal_position(initial_xy)


def _apply_initial_attitude(
    *,
    args: Any,
    base_state: Any,
    initial_bar30: InitialBar30Depth,
) -> None:
    initial_rpy = initial_rpy_vector(args)
    if initial_rpy is None:
        return
    base_state.set_base_attitude_rpy(
        float(initial_rpy[0]),
        float(initial_rpy[1]),
        float(initial_rpy[2]),
    )
    if initial_bar30.value_m is not None:
        base_state.set_bar30_depth(float(initial_bar30.value_m))
    log_initial_attitude(initial_rpy)


__all__ = ["apply_initial_position_and_attitude"]
