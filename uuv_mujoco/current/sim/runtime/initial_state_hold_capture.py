"""Initial-depth hold pose capture helpers."""

from __future__ import annotations

from typing import Any

from sim.runtime.initial_state_depths import InitialBar30Depth


def capture_configured_initial_depth_hold(
    *,
    args: Any,
    initial_depth_hold: Any,
    data: Any,
    base_state: Any,
    initial_bar30: InitialBar30Depth,
) -> None:
    if initial_bar30.value_m is not None:
        _capture_initial_depth_hold_pose(
            initial_depth_hold=initial_depth_hold,
            data=data,
            base_state=base_state,
        )
    elif args.initial_depth_m is not None:
        _capture_initial_depth_hold_pose(
            initial_depth_hold=initial_depth_hold,
            data=data,
            base_state=base_state,
            depth_m=float(args.initial_depth_m),
        )


def _capture_initial_depth_hold_pose(
    *,
    initial_depth_hold: Any,
    data: Any,
    base_state: Any,
    depth_m: float | None = None,
) -> None:
    initial_depth_hold.capture_pose(
        data=data,
        world_qpos_adr=base_state.world_qpos_adr,
        water_surface_z=base_state.water_surface_z,
        depth_m=depth_m,
    )


__all__ = ["capture_configured_initial_depth_hold"]
