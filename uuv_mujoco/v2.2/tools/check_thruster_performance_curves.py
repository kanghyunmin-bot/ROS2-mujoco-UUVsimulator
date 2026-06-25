#!/usr/bin/env python3
"""Smoke checks for thruster performance curve parsing."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TOOLS_DIR = pathlib.Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from thruster_performance_curve_smoke_cases import (  # noqa: E402
    check_curve_parse_sort_and_reject_single_point,
    check_float_array_rejects_invalid_items,
    check_nearest_voltage_selection,
)


def main() -> int:
    check_float_array_rejects_invalid_items()
    check_curve_parse_sort_and_reject_single_point()
    check_nearest_voltage_selection()
    print("thruster_performance_curves=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
