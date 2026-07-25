"""Compatibility surface for thruster PWM-to-force performance config."""

from __future__ import annotations

from sim.physics.thruster_performance_config import (
    default_thruster_performance_config,
    normalize_thruster_perf_voltage,
)
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.physics.thruster_performance_selector import select_thruster_performance_config

_normalize_thruster_perf_voltage = normalize_thruster_perf_voltage

__all__ = [
    "_normalize_thruster_perf_voltage",
    "default_thruster_performance_config",
    "load_thruster_performance_config",
    "select_thruster_performance_config",
]
