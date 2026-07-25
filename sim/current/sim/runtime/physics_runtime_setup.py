"""Compatibility facade for physics, thruster, and underwater-wrench wiring."""

from __future__ import annotations

from sim.runtime.physics_runtime_factory import create_runtime_physics_setup
from sim.runtime.physics_runtime_types import HydrostaticContext, RuntimePhysicsSetup


__all__ = [
    "HydrostaticContext",
    "RuntimePhysicsSetup",
    "create_runtime_physics_setup",
]
