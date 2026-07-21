#!/usr/bin/env python3
"""Smoke checks for internal SITL command override handling."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TOOLS_DIR = pathlib.Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from ros2_sitl_command_override_smoke_cases import run_ros2_sitl_command_override_smoke  # noqa: E402


def main() -> int:
    run_ros2_sitl_command_override_smoke()
    print("ros2_sitl_command_override=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
