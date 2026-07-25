"""Typed bundles for thruster-related runtime setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sim.runtime.thruster_param_runtime import ThrusterParameterRuntime


@dataclass(frozen=True)
class ThrusterParamServoRuntime:
    thruster_param_runtime: ThrusterParameterRuntime
    sitl_servo_runtime: Any


__all__ = ["ThrusterParamServoRuntime"]
