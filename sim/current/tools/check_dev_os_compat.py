#!/usr/bin/env python3
"""Check host OS compatibility for native MuJoCo + ArduSub SITL development."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict

from dev_os_compat_common import ROOT, WORKSPACE, CheckResult
from dev_os_compat_run_groups import add_host_target_results, add_runtime_results, add_system_results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Treat missing display as expected")
    parser.add_argument("--require-viewer", action="store_true", help="Require mujoco.viewer import")
    parser.add_argument("--python", help="Runtime Python to verify instead of launcher-style auto-detection")
    parser.add_argument("--mjpython", help="Runtime mjpython to verify for macOS viewer launches")
    parser.add_argument(
        "--target-os",
        default=os.environ.get("UUV_DEV_OS_TARGET", "auto"),
        help="Migration target OS contract to check: auto, macos, linux, or ubuntu",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    parser.add_argument("--strict", action="store_true", help="Exit nonzero on warnings as well as failures")
    return parser.parse_args()


def run_checks(args: argparse.Namespace) -> tuple[str, list[CheckResult]]:
    results: list[CheckResult] = []
    target_system = add_host_target_results(results, target_os=args.target_os)
    add_runtime_results(results, args=args)
    add_system_results(results, args=args, target_system=target_system)
    return target_system, results


def build_payload(target_system: str, results: list[CheckResult]) -> dict[str, object]:
    return {
        "root": str(ROOT),
        "workspace": str(WORKSPACE),
        "target_os": target_system,
        "results": [asdict(result) for result in results],
        "summary": {
            "fail": sum(1 for result in results if result.status == "fail"),
            "warn": sum(1 for result in results if result.status == "warn"),
            "pass": sum(1 for result in results if result.status == "pass"),
        },
    }


def print_payload(payload: dict[str, object], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return
    for result in payload["results"]:
        print(f"{result['status'].upper():4s} {result['name']}: {result['detail']}")
    print(json.dumps(payload["summary"], sort_keys=True))


def main() -> int:
    args = parse_args()
    target_system, results = run_checks(args)
    payload = build_payload(target_system, results)
    print_payload(payload, as_json=bool(args.json))

    summary = payload["summary"]
    if summary["fail"]:
        return 1
    if args.strict and summary["warn"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
