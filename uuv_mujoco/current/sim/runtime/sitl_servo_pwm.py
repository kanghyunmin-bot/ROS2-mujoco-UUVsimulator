"""PWM mapping helpers for SITL/plant-replay servo runtime."""

from __future__ import annotations

from collections.abc import Sequence
from typing import MutableMapping

import numpy as np


def sitl_pwm_to_norm(pwm: int) -> float:
    """Convert ArduSub bidirectional motor PWM to normalized command."""

    if pwm <= 0 or pwm == 65535:
        return 0.0
    return float(np.clip((float(pwm) - 1500.0) / 400.0, -1.0, 1.0))


def copy_servo_pwm_values(destination: list[int], source: Sequence[int]) -> None:
    for idx in range(len(destination)):
        destination[idx] = int(source[idx]) if idx < len(source) else 1500


def reset_thruster_commands(commands: MutableMapping[str, float], thruster_names: Sequence[str]) -> None:
    for thr_name in thruster_names:
        commands[thr_name] = 0.0


def packet_commands_from_pwm(
    *,
    all_thruster_names: Sequence[str],
    raw_map: Sequence[str],
    servo_signs: Sequence[float],
    pwm_values: Sequence[int],
) -> dict[str, float]:
    packet_commands = {thr_name: 0.0 for thr_name in all_thruster_names}
    for idx, thr_name in enumerate(raw_map):
        if idx >= len(pwm_values):
            break
        norm = sitl_pwm_to_norm(int(pwm_values[idx])) * float(servo_signs[idx])
        packet_commands[thr_name] = float(np.clip(norm, -1.0, 1.0))
    return packet_commands


def copy_scaled_commands_to_targets(
    *,
    targets: MutableMapping[str, float],
    commands: MutableMapping[str, float],
    thruster_names: Sequence[str],
    scale: float,
) -> None:
    for name in thruster_names:
        targets[name] = float(np.clip(commands.get(name, 0.0) * float(scale), -1.0, 1.0))


__all__ = [
    "copy_scaled_commands_to_targets",
    "copy_servo_pwm_values",
    "packet_commands_from_pwm",
    "reset_thruster_commands",
    "sitl_pwm_to_norm",
]
