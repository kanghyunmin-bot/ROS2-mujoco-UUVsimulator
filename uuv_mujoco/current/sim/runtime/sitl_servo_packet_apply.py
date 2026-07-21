"""Packet application helpers for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

import time

import numpy as np

from .sitl_servo_pwm import copy_servo_pwm_values, packet_commands_from_pwm, reset_thruster_commands


def apply_servo_packet(runtime: object, pwm_values: list[int]) -> None:
    copy_servo_pwm_values(runtime.pwm_values, pwm_values)
    reset_thruster_commands(runtime.cmd_norm, runtime.all_thruster_names)
    packet_commands = packet_commands_from_pwm(
        all_thruster_names=runtime.all_thruster_names,
        raw_map=runtime.raw_map,
        servo_signs=runtime.servo_signs,
        pwm_values=pwm_values,
    )
    for thr_name, norm in packet_commands.items():
        runtime.cmd_norm[thr_name] = float(np.clip(norm, -1.0, 1.0))
    runtime.last_wall["value"] = time.monotonic()


__all__ = ["apply_servo_packet"]
