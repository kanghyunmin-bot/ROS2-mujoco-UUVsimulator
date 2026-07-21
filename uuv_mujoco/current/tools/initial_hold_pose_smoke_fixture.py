"""Fixtures for initial hold pose smoke checks."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np


class FakeMujoco:
    def __init__(self) -> None:
        self.forward_calls = 0

    def mj_forward(self, _model, _data) -> None:
        self.forward_calls += 1


def fake_hold_data():
    return SimpleNamespace(
        qpos=np.arange(10, dtype=np.float64),
        qvel=np.ones(10, dtype=np.float64),
        qacc=np.ones(10, dtype=np.float64),
    )


def hold_depth_recorders():
    calls: list[tuple[str, float, bool]] = []

    def set_bar30_depth(value: float, *, reset_velocity: bool) -> None:
        calls.append(("bar30", float(value), bool(reset_velocity)))

    def set_base_depth(value: float, *, reset_velocity: bool) -> None:
        calls.append(("base", float(value), bool(reset_velocity)))

    return calls, set_bar30_depth, set_base_depth


def assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


__all__ = ["FakeMujoco", "assert_equal", "fake_hold_data", "hold_depth_recorders"]
