#!/usr/bin/env python3
"""Build and compare OS-independent closed-loop control fingerprints.

The input is the axis_rc_override_check output.  When MuJoCo thruster debug CSV
is supplied, this also aligns thruster/body wrench rows to the same phase
windows using monotonic wall time.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from control_loop_golden_compare_logic import compare_fingerprints
from control_loop_golden_fingerprint import build_fingerprint, load_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--thruster-csv", type=Path)
    parser.add_argument("--baseline-thruster-csv", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--abs-tol", type=float, default=0.02)
    parser.add_argument("--relative-tol", type=float, default=0.15)
    parser.add_argument("--pwm-tol", type=float, default=8.0)
    parser.add_argument("--force-abs-tol", type=float, default=2.0)
    parser.add_argument("--force-rel-tol", type=float, default=0.20)
    parser.add_argument("--torque-abs-tol", type=float, default=0.25)
    parser.add_argument("--torque-rel-tol", type=float, default=0.20)
    return parser.parse_args()


def build_result(args: argparse.Namespace) -> tuple[dict[str, Any], str]:
    candidate_payload = load_json(args.candidate)
    candidate = build_fingerprint(candidate_payload, args.thruster_csv)
    result: dict[str, Any] = {
        "candidate": str(args.candidate),
        "thruster_csv": str(args.thruster_csv) if args.thruster_csv else None,
        "fingerprint": candidate,
    }

    if args.baseline:
        baseline_payload = load_json(args.baseline)
        baseline = build_fingerprint(baseline_payload, args.baseline_thruster_csv)
        result["baseline"] = str(args.baseline)
        result["comparison"] = compare_fingerprints(candidate, baseline, args)
        overall = str(result["comparison"]["overall"])
        return result, overall

    health = str(candidate.get("health", {}).get("overall", "unknown"))
    overall = "fail" if health == "fail" else ("warn" if health == "warn" else "pass")
    result["comparison"] = {
        "overall": overall,
        "failures": [] if overall != "fail" else [{"kind": "candidate_health", "candidate": health}],
        "warnings": [] if overall != "warn" else [{"kind": "candidate_health", "candidate": health}],
        "comparison_count": 0,
    }
    return result, overall


def write_result(path: Path, result: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")


def print_summary(args: argparse.Namespace, result: dict[str, Any], overall: str) -> None:
    candidate = result["fingerprint"]
    phase_count = len(candidate.get("phases", {}))
    has_thr = "yes" if candidate.get("has_thruster_debug") else "no"
    print(f"[control-loop-compare] out={args.out}")
    print(f"[control-loop-compare] phases={phase_count} thruster_debug={has_thr} overall={overall}")
    failures = result["comparison"].get("failures", [])
    for item in failures[:12]:
        print(f"[control-loop-compare] FAIL {item}")
    warnings = result["comparison"].get("warnings", [])
    for item in warnings[:8]:
        print(f"[control-loop-compare] WARN {item}")


def main() -> int:
    args = parse_args()
    result, overall = build_result(args)
    write_result(args.out, result)
    print_summary(args, result, overall)
    return 1 if overall == "fail" else 0


__all__ = [
    "build_fingerprint",
    "build_result",
    "compare_fingerprints",
    "load_json",
    "main",
    "parse_args",
]


if __name__ == "__main__":
    raise SystemExit(main())
