"""Callback bundle types for the MuJoCo physics step runtime."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class StepPhysicsCallbacks:
    thruster_update_due: Callable[[], tuple[bool, float]]
    update_thruster_forces: Callable[[float], None]
    update_propeller_visuals: Callable[[float], None]
    apply_direct_command_targets: Callable[[], tuple[float, float, float, float]]
    apply_underwater_wrench: Callable[[float], None]
    emit_thruster_debug: Callable[[], None]
    enforce_descent_contract: Callable[[], None]


__all__ = ["StepPhysicsCallbacks"]
