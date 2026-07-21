#!/usr/bin/env python3
"""Smoke checks for initial hold pose/depth helpers."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TOOLS_DIR = pathlib.Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from initial_hold_pose_smoke_cases import run_initial_hold_pose_smoke  # noqa: E402


def main() -> int:
    run_initial_hold_pose_smoke()
    print("initial_hold_pose=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
