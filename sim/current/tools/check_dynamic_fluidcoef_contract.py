#!/usr/bin/env python3
"""Smoke tests for dynamic MuJoCo fluidcoef load and timing contracts."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dynamic_fluidcoef_smoke_cases import (  # noqa: E402
    check_axis_weighted_load,
    check_dynamic_update_cadence,
    check_fluidcoef_load_order,
)


def main() -> int:
    check_axis_weighted_load()
    check_fluidcoef_load_order()
    check_dynamic_update_cadence()
    print("dynamic_fluidcoef_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
