"""Smoke cases for dynamic MuJoCo fluidcoef contracts."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from sim.physics.dynamic_fluidcoef_loads import axis_weighted_load, fluidcoef_loads_from_local_velocity
from sim.physics.dynamic_fluidcoef_runtime_update import dynamic_fluidcoef_update_due


def assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1e-9:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def check_axis_weighted_load() -> None:
    assert_close(axis_weighted_load(np.zeros(3), np.ones(3)), 0.0, "zero weights")
    assert_close(
        axis_weighted_load(np.array([1.0, 0.5, 0.2]), np.array([0.1, 0.9, 0.4])),
        0.45,
        "max weighted load",
    )


def check_fluidcoef_load_order() -> None:
    axis_weights = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
        ],
        dtype=np.float64,
    )
    angular_axis_weights = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    loads = fluidcoef_loads_from_local_velocity(
        rel=np.array([0.3, 0.15, 0.06], dtype=np.float64),
        omega=np.array([0.0, 0.3, 0.6], dtype=np.float64),
        axis_weights=axis_weights,
        angular_axis_weights=angular_axis_weights,
        reference_speed_mps=0.3,
        reference_angular_rps=0.6,
    )
    expected = np.array([1.0, 0.5, 0.5, 0.5, np.sqrt(0.25)], dtype=np.float64)
    if not np.allclose(loads, expected, atol=1e-9, rtol=0.0):
        raise AssertionError(f"fluidcoef load order mismatch: expected {expected}, got {loads}")


def check_dynamic_update_cadence() -> None:
    runtime = SimpleNamespace(next_sim_t=-1.0, update_dt=0.05)
    if not dynamic_fluidcoef_update_due(runtime, 0.0):
        raise AssertionError("first dynamic fluidcoef update must be due")
    if dynamic_fluidcoef_update_due(runtime, 0.01):
        raise AssertionError("dynamic fluidcoef update fired before update_dt")
    if not dynamic_fluidcoef_update_due(runtime, 0.05):
        raise AssertionError("dynamic fluidcoef update did not fire at update_dt")


__all__ = [
    "check_axis_weighted_load",
    "check_dynamic_update_cadence",
    "check_fluidcoef_load_order",
]
