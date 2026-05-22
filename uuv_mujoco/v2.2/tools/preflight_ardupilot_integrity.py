#!/usr/bin/env python3
"""Read-only ArduPilot integrity checks for closed-loop MuJoCo validation."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


WATCHED_PATHS = (
    "ArduSub/control_althold.cpp",
    "libraries/AP_Common/missing/fenv.h",
)


def run_git(ardupilot_dir: Path, args: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(
        ["git", "-C", str(ardupilot_dir), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    return proc.returncode, proc.stdout, proc.stderr


def collect_status(ardupilot_dir: Path) -> dict[str, Any]:
    status_rc, status_out, status_err = run_git(ardupilot_dir, ["status", "--short"])
    diff_payload: dict[str, Any] = {}
    for rel_path in WATCHED_PATHS:
        rc, out, err = run_git(ardupilot_dir, ["diff", "--", rel_path])
        diff_payload[rel_path] = {
            "returncode": rc,
            "stderr": err.strip(),
            "dirty": bool(out.strip()),
            "diff": out,
        }
    return {
        "ardupilot_dir": str(ardupilot_dir),
        "git_status": {
            "returncode": status_rc,
            "stderr": status_err.strip(),
            "short": status_out.splitlines(),
        },
        "watched_diffs": diff_payload,
    }


def classify(payload: dict[str, Any]) -> tuple[bool, list[str]]:
    issues: list[str] = []
    failed = False
    status = payload.get("git_status", {})
    status_lines = [str(line) for line in status.get("short", [])]
    if status.get("returncode") != 0:
        issues.append("P0: ArduPilot git status failed; closed-loop validation cannot prove the controller baseline.")
        failed = True

    althold = payload.get("watched_diffs", {}).get("ArduSub/control_althold.cpp", {})
    if althold.get("dirty"):
        diff = str(althold.get("diff", ""))
        if "motors.set_throttle" in diff or "channel_throttle" in diff or "raw_throttle_factor" in diff:
            issues.append(
                "P0: ArduSub/control_althold.cpp modifies ALT_HOLD throttle/heave mapping; "
                "depth/heave closed-loop results are not trustworthy."
            )
        else:
            issues.append("P0: ArduSub/control_althold.cpp is modified; ALT_HOLD baseline is dirty.")
        failed = True

    for rel_path in WATCHED_PATHS:
        if any(line.endswith(rel_path) for line in status_lines):
            if rel_path != "ArduSub/control_althold.cpp":
                issues.append(
                    f"P1: ArduPilot dependency file is modified: {rel_path}; "
                    "allowed for host build compatibility, but record it in the run report."
                )

    return failed, issues


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

    ardupilot_dir = args.ardupilot_dir or (args.workspace / "ardupilot")
    if not ardupilot_dir.exists():
        print(f"[ardupilot-preflight] missing ArduPilot directory: {ardupilot_dir}", file=sys.stderr)
        return 2

    payload = collect_status(ardupilot_dir)
    failed, issues = classify(payload)
    payload["failed"] = failed
    payload["issues"] = issues
    payload["allow_dirty"] = bool(args.allow_dirty)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if issues:
        for issue in issues:
            print(f"[ardupilot-preflight] {issue}", file=sys.stderr)
    else:
        print("[ardupilot-preflight] ArduPilot watched files are clean")

    if failed and not args.allow_dirty:
        print(
            "[ardupilot-preflight] refusing authoritative closed-loop validation. "
            "Set UUV_ALLOW_DIRTY_ARDUPILOT=1 only for smoke tests.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
