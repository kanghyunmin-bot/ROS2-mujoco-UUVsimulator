#!/usr/bin/env python3
"""Verify the one-pass course-buoy contact classifier and force cache."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402


def main() -> int:
    contacts = [
        SimpleNamespace(geom1=10, geom2=101),
        SimpleNamespace(geom1=102, geom2=11),
        SimpleNamespace(geom1=101, geom2=10),
        SimpleNamespace(geom1=101, geom2=102),
    ]
    force_rows = {
        0: (3.0, 4.0, 0.0),
        2: (6.0, 8.0, 0.0),
    }
    calls: list[int] = []

    def contact_force(_model, _data, contact_id: int, out: np.ndarray) -> None:
        calls.append(int(contact_id))
        out[:3] = force_rows[int(contact_id)]

    runtime = object.__new__(CourseBuoyRuntime)
    runtime.model = object()
    runtime.data = SimpleNamespace(ncon=len(contacts), contact=contacts)
    runtime.mujoco_module = SimpleNamespace(mj_contactForce=contact_force)
    runtime.vehicle_geom_ids = frozenset({10, 11, 12})
    runtime._release_probe_geom_id_set = frozenset({10})
    runtime._buoy_body_by_geom = {101: 1, 102: 2}
    runtime._contact_force_scratch = np.zeros(6, dtype=np.float64)

    contacted, rake_contacted, force_by_body = runtime._contact_snapshot()
    if contacted != {1, 2}:
        raise AssertionError(f"all-vehicle contact classification mismatch: {contacted}")
    if rake_contacted != {1}:
        raise AssertionError(f"rake contact classification mismatch: {rake_contacted}")
    if force_by_body != {1: 10.0}:
        raise AssertionError(f"rake peak-force cache mismatch: {force_by_body}")
    if calls != [0, 2]:
        raise AssertionError(f"contact force evaluated outside rake contacts: {calls}")
    if runtime._contacted_buoy_body_ids(runtime.vehicle_geom_ids) != contacted:
        raise AssertionError("one-pass all-vehicle result differs from compatibility oracle")
    if runtime._contacted_buoy_body_ids(runtime._release_probe_geom_id_set) != rake_contacted:
        raise AssertionError("one-pass rake result differs from compatibility oracle")

    print("course_buoy_contact_snapshot=PASS ncon=4 scans=1 force_calls=2 peak=10.0N")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
