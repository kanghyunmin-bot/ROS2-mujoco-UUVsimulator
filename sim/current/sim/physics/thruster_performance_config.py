"""Shared config helpers for thruster performance curves."""

from __future__ import annotations

import numpy as np


def default_thruster_performance_config(
    *,
    requested_voltage: float,
    direct: bool,
) -> dict:
    return {
        "active": False,
        "direct": bool(direct),
        "requested_voltage": float(requested_voltage),
        "selected_voltage": None,
        "pwm": np.array([], dtype=np.float64),
        "force": np.array([], dtype=np.float64),
    }


def normalize_thruster_perf_voltage(value: float | str | None, default_voltage: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default_voltage)


__all__ = ["default_thruster_performance_config", "normalize_thruster_perf_voltage"]
