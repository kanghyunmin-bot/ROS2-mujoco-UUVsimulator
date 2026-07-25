#!/usr/bin/env python3
"""Smoke checks for real-start measurement calculations."""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from real_start_measurement_smoke_cases import (  # noqa: E402
    check_bar30_depth_contract,
    check_base_link_depth_contract,
)


def main() -> int:
    check_bar30_depth_contract()
    check_base_link_depth_contract()
    print("real_start_measurements=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
