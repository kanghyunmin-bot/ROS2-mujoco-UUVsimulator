#!/usr/bin/env python3
"""Smoke checks for SITL ExternalNav contract enforcement."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

TOOLS_DIR = pathlib.Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_external_nav_contract_cases import (  # noqa: E402
    check_native_vpd_freshness,
    check_required_disabled_fault_and_grace,
    check_tx_missing_and_stale,
)


def main() -> int:
    check_required_disabled_fault_and_grace()
    check_tx_missing_and_stale()
    check_native_vpd_freshness()
    print("sitl_external_nav_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
