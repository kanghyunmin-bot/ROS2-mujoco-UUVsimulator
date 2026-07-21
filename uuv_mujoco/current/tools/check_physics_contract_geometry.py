#!/usr/bin/env python3
"""Dependency-light smoke for physics contract geometry helpers."""

from __future__ import annotations

from pathlib import Path
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
for item in (ROOT, TOOLS):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))

from physics_contract_quat import normalized_quat_wxyz, rpy_rad_from_quat_wxyz  # noqa: E402


def main() -> int:
    assert normalized_quat_wxyz(np.array([0.0, 0.0, 0.0, 0.0])) is None
    assert rpy_rad_from_quat_wxyz(np.array([1.0, 0.0, 0.0, 0.0])) == (0.0, 0.0, 0.0)
    _roll, _pitch, yaw = rpy_rad_from_quat_wxyz(
        np.array([math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)])
    )
    assert abs(yaw - math.pi / 2.0) < 1.0e-9
    print("physics_contract_geometry=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
