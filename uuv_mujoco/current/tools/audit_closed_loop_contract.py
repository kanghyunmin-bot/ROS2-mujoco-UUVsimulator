#!/usr/bin/env python3
"""Emit the closed-loop validation contract used by SITL/MuJoCo replay runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from audit_closed_loop_params import (
    WATCH_PARAMS,
    parse_param_file,
    parse_start_sitl_enforced_params,
    same_param_value,
)
from audit_closed_loop_payload import build_contract_payload
from audit_closed_loop_profile import (
    CURRENT_INACTIVE_KEYS,
    PROFILE_KEYS,
    load_profile,
    nested_get,
    resolve_active_runtime,
    selected_thruster_curve,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--profile", default="current")
    parser.add_argument(
        "--sitl-log",
        type=Path,
        default=None,
        help="Optional start_ardusub_sitl log; used to capture temp enforced params.",
    )
    parser.add_argument("--json-out", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = build_contract_payload(
        workspace=args.workspace,
        profile_name=args.profile,
        sitl_log=args.sitl_log,
    )
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[contract-audit] wrote {args.json_out}")

    mismatches = payload.get("real_vs_sitl_mismatches", {})
    if mismatches:
        print(f"[contract-audit] real/SITL watched param mismatches: {len(mismatches)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CURRENT_INACTIVE_KEYS",
    "PROFILE_KEYS",
    "WATCH_PARAMS",
    "build_contract_payload",
    "load_profile",
    "main",
    "nested_get",
    "parse_args",
    "parse_param_file",
    "parse_start_sitl_enforced_params",
    "resolve_active_runtime",
    "same_param_value",
    "selected_thruster_curve",
]
