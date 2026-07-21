#!/usr/bin/env python3
"""Audit the static SERVO_OUTPUT_RAW -> MuJoCo wrench contract.

This tool does not tune hydrodynamics. It checks the deterministic contract
that maps ArduSub VECTORED_6DOF motor axes into the current MuJoCo actuator
sites/gears and per-thruster direct gains.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from actuator_wrench_calc import build_actuator_wrench_result
from actuator_wrench_common import ROOT
from actuator_wrench_report import write_markdown


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=ROOT / "scenes" / "tank_current_scene.xml")
    parser.add_argument("--profile-file", type=Path, default=ROOT / "config" / "sim_profiles.json")
    parser.add_argument("--profile", default="current")
    parser.add_argument("--thruster-params", type=Path, default=ROOT / "config" / "thruster_params.json")
    parser.add_argument("--horizontal-z-offset", type=float, default=None)
    parser.add_argument("--vertical-x-scale", type=float, default=None)
    parser.add_argument("--unit-gains", action="store_true")
    parser.add_argument("--out-json", type=Path, default=None)
    parser.add_argument("--out-md", type=Path, default=None)
    args = parser.parse_args()

    result = build_actuator_wrench_result(
        scene=args.scene,
        profile_file=args.profile_file,
        profile_name=args.profile,
        thruster_params=args.thruster_params,
        horizontal_z_offset=args.horizontal_z_offset,
        vertical_x_scale=args.vertical_x_scale,
        unit_gains=bool(args.unit_gains),
    )

    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    if args.out_md:
        args.out_md.parent.mkdir(parents=True, exist_ok=True)
        write_markdown(args.out_md, result)
    print(json.dumps(result["axes"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
