"""Factory helpers for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

import numpy as np


def build_sitl_servo_runtime_kwargs(
    *,
    all_thruster_names: list[str],
    raw_map: list[str],
    servo_signs: list[float],
    sitl_servo_scale: float,
    timeout_s: float,
) -> dict[str, object]:
    return {
        "all_thruster_names": list(all_thruster_names),
        "raw_map": list(raw_map),
        "servo_signs": [float(value) for value in servo_signs],
        "cmd_norm": {name: 0.0 for name in all_thruster_names},
        "pwm_values": [1500] * 8,
        "last_wall": {"value": -1.0},
        "scale": float(np.clip(sitl_servo_scale, 0.0, 2.0)),
        "timeout_s": float(max(timeout_s, 0.0)),
    }


__all__ = ["build_sitl_servo_runtime_kwargs"]
