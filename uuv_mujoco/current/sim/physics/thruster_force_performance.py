"""T200/direct performance-curve force conversion."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
)

_NATIVE_PWM_TO_SCENE_DIRECTIONS = dict(zip(
    ARDUSUB_VECTORED_6DOF_SERVO_MAP, ARDUSUB_VECTORED_6DOF_SERVO_SIGNS
))
ESC_DEADBAND_US = 25.0


def native_pwm_direction(name: str) -> float:
    """Return the direction between the scene actuator and the native ESC curve."""
    return float(_NATIVE_PWM_TO_SCENE_DIRECTIONS.get(name, 1.0))


def rotor_rpm_from_command(name: str, command: float, perf_cfg: Mapping[str, Any]) -> float:
    """Return steady, signed rotor speed [RPM] for the scene-axis PWM command."""
    direction = native_pwm_direction(name)
    pwm = 1500.0 + 400.0 * direction * float(np.clip(command, -1.0, 1.0))
    if abs(pwm - 1500.0) <= ESC_DEADBAND_US:
        return 0.0
    return direction * float(np.interp(pwm, perf_cfg["pwm"], perf_cfg["rpm"]))


def pwm_to_force_from_performance(norm_cmd: float, perf_cfg: Mapping[str, Any]) -> float:
    """Map normalized command [-1, 1] to force using a PWM performance curve."""
    pwm = float(np.clip(norm_cmd, -1.0, 1.0) * 400.0 + 1500.0)
    # Basic ESC signal deadband. Do not renormalize the remaining PWM range:
    # the measured curve already contains ESC/motor startup behavior.
    if abs(pwm - 1500.0) <= ESC_DEADBAND_US:
        return 0.0
    return float(np.interp(pwm, perf_cfg["pwm"], perf_cfg["force"]))


def performance_curve_active(perf_cfg: Mapping[str, Any]) -> bool:
    force_curve = perf_cfg.get("force")
    return bool(perf_cfg.get("active") and getattr(force_curve, "size", 0) > 0)


def force_from_performance_curve(
    *,
    name: str,
    command_shaped: float,
    gain: float,
    perf_cfg: Mapping[str, Any],
    thruster_direct_scale: Mapping[str, float],
) -> float:
    if perf_cfg.get("direct"):
        # Commands already use the scene actuator axis. Restore the native
        # ESC polarity for the asymmetric measured curve, then rotate the
        # resulting force into that scene axis. This never reapplies MOT_DIR.
        direction = native_pwm_direction(name)
        return float(
            direction * pwm_to_force_from_performance(command_shaped * direction, perf_cfg)
            * float(thruster_direct_scale.get(name, 1.0))
        )
    return float(pwm_to_force_from_performance(command_shaped, perf_cfg) * gain)


__all__ = [
    "force_from_performance_curve",
    "performance_curve_active",
    "pwm_to_force_from_performance",
]
