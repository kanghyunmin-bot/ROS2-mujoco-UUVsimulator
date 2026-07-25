#!/usr/bin/env python3
"""Smoke-check simulation-profile buoyancy point parsing."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.sim_profile_hydrostatic_buoyancy_points import parse_buoyancy_points  # noqa: E402


def main() -> int:
    raw_pos = [0.1, -0.2, 0.3]
    points = parse_buoyancy_points(
        {
            "buoyancy_points": [
                {"name": "front", "pos": raw_pos, "share": 2.0},
                {"name": "bad_share", "pos": [0, 0, 0], "share": -1.0},
                {"name": "bad_height", "pos": [0, 0, 0], "share": 1.0, "half_height": 0.0},
                {"name": "bad_pos", "pos": [0, 0], "share": 1.0},
                {"pos": [0.0, 0.0, 0.0], "share": 0.0, "half_height": 0.25},
                "not-a-mapping",
            ]
        },
        default_half_height=0.5,
    )
    assert len(points) == 2
    assert points[0].name == "front"
    assert points[0].share == 2.0
    assert points[0].half_height == 0.5
    assert points[1].name == "buoyancy_point_4"
    assert points[1].half_height == 0.25
    np.testing.assert_allclose(points[0].pos, np.array(raw_pos, dtype=np.float64))
    raw_pos[0] = 99.0
    assert points[0].pos[0] == 0.1
    print("hydrostatic_buoyancy_points=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
