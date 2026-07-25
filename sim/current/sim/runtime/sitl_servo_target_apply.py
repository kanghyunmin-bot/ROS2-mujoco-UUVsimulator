"""Target application helpers for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

from .sitl_servo_pwm import copy_scaled_commands_to_targets, reset_thruster_commands


def servo_packet_is_stale(runtime: object, *, now_wall: float, timeout_s: float) -> bool:
    last_wall = float(runtime.last_wall["value"])
    return last_wall <= 0.0 or (float(now_wall) - last_wall) > float(timeout_s)


def apply_servo_commands_to_targets(
    runtime: object,
    targets: dict[str, float],
    *,
    now_wall: float,
    timeout_s: float,
) -> bool:
    if servo_packet_is_stale(runtime, now_wall=now_wall, timeout_s=timeout_s):
        reset_thruster_commands(targets, runtime.all_thruster_names)
        return True
    copy_scaled_commands_to_targets(
        targets=targets,
        commands=runtime.cmd_norm,
        thruster_names=runtime.all_thruster_names,
        scale=runtime.scale,
    )
    return False


__all__ = ["apply_servo_commands_to_targets", "servo_packet_is_stale"]
