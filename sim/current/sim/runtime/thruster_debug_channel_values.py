"""Per-channel row values for MuJoCo thruster debug output."""

from __future__ import annotations


def append_servo_pwm_values(values: list[float], sitl_servo_pwm_values: list[int]) -> None:
    values.extend(float(value) for value in sitl_servo_pwm_values[:8])


def append_thruster_command_values(
    values: list[float],
    *,
    thruster_names: list[str],
    sitl_servo_cmd_norm: dict[str, float],
    thr_target: dict[str, float],
    thr_state: dict[str, float],
    thruster_force_cmd: dict[str, float],
    thruster_direct_scale: dict[str, float],
) -> None:
    for thr_name in thruster_names:
        values.extend(
            [
                float(sitl_servo_cmd_norm.get(thr_name, 0.0)),
                float(thr_target.get(thr_name, 0.0)),
                float(thr_state.get(thr_name, 0.0)),
                float(thruster_force_cmd.get(thr_name, 0.0)),
                float(thruster_direct_scale.get(thr_name, 1.0)),
            ]
        )


__all__ = ["append_servo_pwm_values", "append_thruster_command_values"]
