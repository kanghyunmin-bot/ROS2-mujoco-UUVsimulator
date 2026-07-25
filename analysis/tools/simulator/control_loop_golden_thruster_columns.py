"""Column selection helpers for golden control-loop thruster summaries."""

from __future__ import annotations


THRUSTER_BODY_COLUMNS = (
    "thr_force_body_x",
    "thr_force_body_y",
    "thr_force_body_z",
    "thr_torque_body_x",
    "thr_torque_body_y",
    "thr_torque_body_z",
)


def sitl_pwm_columns(fieldnames: list[str]) -> list[str]:
    return [f"sitl_ch{i}_pwm" for i in range(1, 9) if f"sitl_ch{i}_pwm" in fieldnames]


def interesting_thruster_columns(fieldnames: list[str]) -> list[str]:
    servo_norm_columns = [name for name in fieldnames if name.endswith("_servo_norm")]
    thruster_force_columns = [
        name
        for name in fieldnames
        if name.endswith("_force") and not name.startswith("thr_force_body_")
    ]
    interesting = (
        list(THRUSTER_BODY_COLUMNS)
        + sitl_pwm_columns(fieldnames)
        + servo_norm_columns
        + thruster_force_columns
    )
    return [name for name in interesting if name in fieldnames]


__all__ = ["THRUSTER_BODY_COLUMNS", "interesting_thruster_columns", "sitl_pwm_columns"]
