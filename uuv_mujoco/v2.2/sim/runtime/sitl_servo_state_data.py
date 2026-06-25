"""State container for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SitlServoRuntimeState:
    all_thruster_names: list[str]
    raw_map: list[str]
    servo_signs: list[float]
    cmd_norm: dict[str, float]
    pwm_values: list[int]
    last_wall: dict[str, float]
    scale: float
    timeout_s: float = 0.8


__all__ = ["SitlServoRuntimeState"]
