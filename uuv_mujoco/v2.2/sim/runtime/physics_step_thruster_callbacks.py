"""Thruster command callbacks for one MuJoCo physics step."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from sim.physics.horizontal_allocator import HorizontalAllocator


@dataclass
class ThrusterStepCallbacks:
    thruster_update_due: Callable[[], tuple[bool, float]]
    update_thruster_forces: Callable[[float], None]
    update_propeller_visuals: Callable[[float], None]
    apply_direct_command_targets: Callable[[], tuple[float, float, float, float]]


def build_thruster_step_callbacks(
    *,
    model: Any,
    data: Any,
    base_id: int,
    command_state: Any,
    horizontal_order: list[str],
    vertical_names: list[str],
    horizontal_allocator: HorizontalAllocator,
    hydro_runtime: Any,
    thruster_actuator_runtime: Any,
) -> ThrusterStepCallbacks:
    def thruster_update_due() -> tuple[bool, float]:
        return hydro_runtime.thruster_scheduler.due(float(data.time), fallback_dt=float(model.opt.timestep))

    def update_thruster_forces(dt: float) -> None:
        thruster_actuator_runtime.update_forces(dt, base_id=base_id)

    def update_propeller_visuals(dt: float) -> None:
        thruster_actuator_runtime.update_propeller_visuals(dt)

    def mix_horizontal_thrusters(fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
        return horizontal_allocator.mix(fwd_cmd, sway_cmd, yaw_cmd)

    def apply_direct_command_targets() -> tuple[float, float, float, float]:
        return thruster_actuator_runtime.apply_direct_command_targets(
            command_state=command_state,
            mix_horizontal_thrusters=mix_horizontal_thrusters,
            vertical_names=vertical_names,
            horizontal_order=horizontal_order,
        )

    return ThrusterStepCallbacks(
        thruster_update_due=thruster_update_due,
        update_thruster_forces=update_thruster_forces,
        update_propeller_visuals=update_propeller_visuals,
        apply_direct_command_targets=apply_direct_command_targets,
    )


__all__ = ["ThrusterStepCallbacks", "build_thruster_step_callbacks"]
