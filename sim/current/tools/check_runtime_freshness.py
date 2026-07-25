#!/usr/bin/env python3
"""Check that the active MuJoCo runtime is current before launching."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from runtime_freshness_eval import evaluate_freshness
from runtime_freshness_probe import DEFAULT_REMOTE_REF, collect_freshness_inputs, default_workspace
from runtime_freshness_report import print_json_report, print_text_report
from runtime_freshness_version import refresh_runtime_version


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=default_workspace())
    parser.add_argument("--runtime-dir", type=Path, default=None)
    parser.add_argument("--remote-ref", default=DEFAULT_REMOTE_REF)
    parser.add_argument("--fetch", action="store_true", help="fetch origin before comparing remote ref")
    parser.add_argument(
        "--refresh-version",
        action="store_true",
        help="refresh uuv_mujoco/RUNTIME_VERSION.json when freshness checks pass",
    )
    parser.add_argument("--json", action="store_true", help="write machine-readable JSON")
    parser.add_argument("--warn-only", action="store_true", help="always exit 0 after reporting")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    inputs = collect_freshness_inputs(
        workspace=args.workspace.resolve(),
        remote_ref=args.remote_ref,
        runtime_dir=args.runtime_dir,
        fetch=args.fetch,
    )
    report = evaluate_freshness(inputs)
    if args.refresh_version:
        refreshed_version, refresh_result = refresh_runtime_version(inputs, report)
        inputs["runtime_version"] = refreshed_version
        report = evaluate_freshness(inputs)
        report["version_refresh"] = refresh_result
    if args.json:
        print_json_report(report)
    else:
        print_text_report(report)
    if args.warn_only:
        return 0
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
