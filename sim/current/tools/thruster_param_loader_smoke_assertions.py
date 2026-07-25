"""Assertions for thruster parameter loader smoke checks."""

from __future__ import annotations


def assert_loaded_thruster_param_maps(maps: dict[str, dict]) -> None:
    if maps["scale"] != {"t1": 3.0, "t2": 1.0}:
        raise AssertionError(f"scale mismatch: {maps['scale']}")
    if maps["direct"] != {"t1": 1.5, "t2": 1.0}:
        raise AssertionError(f"direct scale mismatch: {maps['direct']}")
    if maps["reverse"]["t1"] != 0.8 or maps["reverse"]["t2"] is not None:
        raise AssertionError(f"reverse asymmetry mismatch: {maps['reverse']}")
    if maps["tau_up"]["t1"] != 0.2 or maps["tau_down"]["t1"] is not None:
        raise AssertionError("tau override mismatch")
    if maps["global"].get("deadzone") != 0.04:
        raise AssertionError(f"global params mismatch: {maps['global']}")


__all__ = ["assert_loaded_thruster_param_maps"]
