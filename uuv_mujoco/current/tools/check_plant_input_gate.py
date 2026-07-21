#!/usr/bin/env python3
"""Fail fast on invalid plant-input evidence from a full MuJoCo/SITL run."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

V22_ROOT = Path(__file__).resolve().parents[1]
if str(V22_ROOT) not in sys.path:
    sys.path.insert(0, str(V22_ROOT))

from sim.validation.plant_input_gate import evaluate_plant_input_gate  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rcout-csv", type=Path, required=True)
    parser.add_argument("--runtime-log", type=Path, action="append", default=[])
    parser.add_argument("--require-rows", type=int, default=1)
    parser.add_argument("--require-non-neutral", action="store_true")
    parser.add_argument(
        "--strict-log-signatures",
        action="store_true",
        help="Fail on disarmed/neutral JSON-servo log signatures even when CSV activity is present.",
    )
    parser.add_argument("--neutral-center", type=float, default=1500.0)
    parser.add_argument("--neutral-tol", type=float, default=2.0)
    args = parser.parse_args()

    result = evaluate_plant_input_gate(
        csv_path=args.rcout_csv.expanduser(),
        log_paths=[path.expanduser() for path in args.runtime_log],
        require_rows=int(args.require_rows),
        require_non_neutral=bool(args.require_non_neutral),
        neutral_center=float(args.neutral_center),
        neutral_tolerance=float(args.neutral_tol),
        strict_log_signatures=bool(args.strict_log_signatures),
    )
    print(result.to_json())
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
