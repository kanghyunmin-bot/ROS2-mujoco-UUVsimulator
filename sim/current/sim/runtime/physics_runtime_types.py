"""Shared records for physics runtime setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from sim.physics.horizontal_allocator import HorizontalAllocator
from sim.runtime.thruster_actuator_runtime import ThrusterActuatorRuntime
from sim.runtime.thruster_debug_runtime import ThrusterDebugRuntime
from sim.runtime.underwater_wrench_runtime import UnderwaterWrenchRuntime


@dataclass
class RuntimePhysicsSetup:
    sitl_servo_runtime: Any
    sitl_servo_pwm_values: list[int]
    sitl_servo_timeout_s: float
    thruster_actuator_runtime: ThrusterActuatorRuntime
    thruster_debug_runtime: ThrusterDebugRuntime
    underwater_wrench_runtime: UnderwaterWrenchRuntime
    horizontal_allocator: HorizontalAllocator
    thruster_site_ids: dict[str, int]
    thr_state: dict[str, float]
    thr_target: dict[str, float]
    thruster_force_max: float
    vehicle_mass: float
    gravity: float
    thruster_update_due: Callable[[], tuple[bool, float]]
    update_thruster_forces: Callable[[float], None]
    update_propeller_visuals: Callable[[float], None]
    apply_direct_command_targets: Callable[[], tuple[float, float, float, float]]
    apply_underwater_wrench: Callable[[float], None]
    emit_thruster_debug: Callable[[], None]
    enforce_descent_contract: Callable[[], None]


@dataclass
class HydrostaticContext:
    hydro_cfg: Any
    hydrostatic_runtime: Any
    horizontal_allocator: HorizontalAllocator
    rho: float
    gravity: float
    vehicle_mass: float
    neutral_volume: float


__all__ = ["HydrostaticContext", "RuntimePhysicsSetup"]
