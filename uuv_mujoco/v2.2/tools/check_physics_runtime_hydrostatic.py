#!/usr/bin/env python3
"""Smoke tests for runtime hydrostatic config and mass-reference helpers."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.physics_runtime_hydro_config import performance_force_max
from sim.runtime.physics_runtime_mass_reference import build_mass_reference


def _fake_model(masses: list[float]) -> SimpleNamespace:
    return SimpleNamespace(
        opt=SimpleNamespace(gravity=np.array([0.0, 0.0, -9.81], dtype=np.float64)),
        body_mass=np.array(masses, dtype=np.float64),
    )


def _assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1e-9:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def main() -> int:
    assert performance_force_max(np, {"active": False, "force": np.array([3.0])}) is None
    assert performance_force_max(np, {"active": True, "force": np.array([])}) is None
    _assert_close(
        performance_force_max(np, {"active": True, "force": np.array([-2.0, 3.5])}),
        3.5,
        "performance_force_max",
    )

    logs: list[str] = []
    mass_ref = build_mass_reference(
        np_module=np,
        model=_fake_model([0.0, 10.0, 2.0, 3.0]),
        base_id=1,
        scene_fluid_density=1000.0,
        log=logs.append,
        subtree_mass_fn=lambda _model, _base_id: 10.0,
    )
    _assert_close(mass_ref.gravity, 9.81, "gravity")
    _assert_close(mass_ref.vehicle_mass, 10.0, "vehicle_mass")
    _assert_close(mass_ref.neutral_volume, 0.01, "neutral_volume")
    if not logs:
        raise AssertionError("expected all_nonworld mass-reference log")

    fallback_ref = build_mass_reference(
        np_module=np,
        model=_fake_model([0.0, 7.0]),
        base_id=1,
        scene_fluid_density=997.0,
        log=lambda _msg: None,
        subtree_mass_fn=lambda _model, _base_id: 0.0,
    )
    _assert_close(fallback_ref.vehicle_mass, 7.0, "fallback vehicle_mass")
    _assert_close(fallback_ref.neutral_volume, 7.0 / 997.0, "fallback neutral_volume")
    print("physics_runtime_hydrostatic=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
