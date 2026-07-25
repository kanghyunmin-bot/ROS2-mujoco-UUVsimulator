#!/usr/bin/env python3
"""Read-only ArduPilot integrity checks for closed-loop MuJoCo validation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from preflight_ardupilot_classify import classify
from preflight_ardupilot_git import collect_status
from preflight_ardupilot_output import preflight_exit_code, print_preflight_result, write_json_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--ardupilot-dir", type=Path, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="Report dirty state but return success. Use only for non-authoritative smoke checks.",
    )
    args = parser.parse_args()

    ardupilot_dir = args.ardupilot_dir or (args.workspace / "sim" / "ardupilot")
    if not ardupilot_dir.exists():
        print(f"[ardupilot-preflight] missing ArduPilot directory: {ardupilot_dir}", file=sys.stderr)
        return 2

    payload = collect_status(ardupilot_dir)
    failed, issues = classify(payload)
    payload["failed"] = failed
    payload["issues"] = issues
    payload["allow_dirty"] = bool(args.allow_dirty)

    write_json_report(args.json_out, payload)
    print_preflight_result(issues)
    return preflight_exit_code(failed=failed, allow_dirty=bool(args.allow_dirty))


if __name__ == "__main__":
    raise SystemExit(main())
