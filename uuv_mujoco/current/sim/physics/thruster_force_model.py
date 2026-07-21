"""Thruster command-to-force helpers shared by runtime and validation tools."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from sim.physics.thruster_force_performance import (
    force_from_performance_curve,
    performance_curve_active,
    pwm_to_force_from_performance,
)
from sim.physics.thruster_force_polynomial import force_from_polynomial_model


def force_from_shaped_command(
    *,
    name: str,
    command_shaped: float,
    gain: float,
    perf_cfg: Mapping[str, Any],
    thruster_direct_scale: Mapping[str, float],
    thruster_global: Mapping[str, Any],
    thruster_force_max: float,
    thruster_reverse_asymmetry: Mapping[str, float | None],
) -> float:
    """Convert a shaped normalized command into actuator-positive force."""
    if abs(command_shaped) <= 1.0e-9:
        return 0.0

    if performance_curve_active(perf_cfg):
        return force_from_performance_curve(
            name=name,
            command_shaped=command_shaped,
            gain=gain,
            perf_cfg=perf_cfg,
            thruster_direct_scale=thruster_direct_scale,
        )

    return force_from_polynomial_model(
        name=name,
        command_shaped=command_shaped,
        gain=gain,
        thruster_global=thruster_global,
        thruster_force_max=thruster_force_max,
        thruster_reverse_asymmetry=thruster_reverse_asymmetry,
    )


__all__ = ["force_from_shaped_command", "pwm_to_force_from_performance"]
