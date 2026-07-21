"""Smoke cases for thruster performance curve parsing."""

from __future__ import annotations

import numpy as np

from physics.thruster_performance_curves import (
    parse_thruster_performance_candidates,
    parse_thruster_performance_curve,
    select_nearest_thruster_performance_candidate,
    to_float_array,
)


def check_float_array_rejects_invalid_items() -> None:
    if to_float_array(["1", "bad"]) is not None:
        raise AssertionError("invalid float array should be rejected")


def check_curve_parse_sort_and_reject_single_point() -> None:
    candidate = parse_thruster_performance_curve(
        {"voltage_v": "20.0", "pwm_us": [1600, 1500, 1700], "force_n": [2.0, 0.0, 4.0]}
    )
    if candidate is None:
        raise AssertionError("valid candidate was rejected")
    if float(candidate["voltage"]) != 20.0:
        raise AssertionError("voltage parse mismatch")
    if not np.allclose(candidate["pwm"], np.array([1500.0, 1600.0, 1700.0])):
        raise AssertionError(f"PWM sort mismatch: {candidate['pwm']}")
    if parse_thruster_performance_curve({"voltage_v": 20.0, "pwm_us": [1500], "force_n": [0.0]}) is not None:
        raise AssertionError("single-point curve should be rejected")


def check_nearest_voltage_selection() -> None:
    candidates = parse_thruster_performance_candidates(
        [
            None,
            {"voltage_v": 16.0, "pwm_us": [1500, 1600], "force_n": [0.0, 1.0]},
            {"voltage_v": 22.0, "pwm_us": [1500, 1600], "force_n": [0.0, 3.0]},
        ]
    )
    nearest = select_nearest_thruster_performance_candidate(candidates, 21.0)
    if float(nearest["voltage"]) != 22.0:
        raise AssertionError("nearest voltage selection mismatch")


__all__ = [
    "check_curve_parse_sort_and_reject_single_point",
    "check_float_array_rejects_invalid_items",
    "check_nearest_voltage_selection",
]
