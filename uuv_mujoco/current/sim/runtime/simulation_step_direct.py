"""Direct-command step path for the MuJoCo runtime."""

from __future__ import annotations

from typing import Any

from .simulation_step_physics import apply_common_step_physics


def run_direct_command_runtime_step(
    runtime: Any,
    *,
    is_paused: bool,
    thruster_due: bool,
    thruster_dt: float,
    publish_ros: bool,
) -> tuple[float, float, float, float]:
    forward, sway, yaw, heave = runtime.apply_direct_command_targets()
    apply_common_step_physics(
        runtime,
        is_paused=is_paused,
        thruster_due=thruster_due,
        thruster_dt=thruster_dt,
        auto_release_initial_hold=False,
    )
    if publish_ros:
        runtime.publish_ros_once()
    return forward, sway, yaw, heave


__all__ = ["run_direct_command_runtime_step"]
