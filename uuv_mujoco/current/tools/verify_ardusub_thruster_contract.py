#!/usr/bin/env python3
"""Verify the ArduSub VECTORED_6DOF -> MuJoCo thruster contract.

The check uses three independent pieces of data:
1) ArduSub's VECTORED_6DOF mixer factors.
2) The real vehicle MOT_x_DIRECTION values.
3) The current MuJoCo scene's thruster site positions and gear vectors.

It fails if a positive ArduSub axis command produces the wrong primary FRD
wrench sign in MuJoCo.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ardusub_thruster_contract_constants import AXES  # noqa: E402
from ardusub_thruster_contract_response import build_axis_response  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "scenes" / "tank_current_scene.xml",
        help="MuJoCo scene XML to verify.",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    report = build_axis_response(args.scene)
    failed = [axis for axis, item in report.items() if not bool(item["ok"])]
    if args.json:
        print(json.dumps({"ok": not failed, "failed": failed, "axes": report}, indent=2))
    elif not args.quiet:
        print("[thruster-contract] ArduSub VECTORED_6DOF -> MuJoCo primary FRD response")
        for axis in AXES:
            item = report[axis]
            print(f"  {axis:8s}: primary={float(item['primary']):+.6f} ok={bool(item['ok'])}")
    if failed:
        print(
            "[thruster-contract] failed axes: " + ", ".join(failed),
            file=sys.stderr,
        )
        return 1
    if not args.json:
        print("[thruster-contract] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
