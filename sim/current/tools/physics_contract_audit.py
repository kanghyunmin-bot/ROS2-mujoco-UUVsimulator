#!/usr/bin/env python3
"""Audit the MuJoCo UUV static hydrostatic physics contract.

This intentionally does not touch ArduSub. It checks whether the MuJoCo plant
is neutrally buoyant, fully submerged at the configured start pose, and whether
neutral PWM implies zero thruster force before any controller gain is used to
hide a plant bias.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return float(default)
    try:
        return float(value)
    except ValueError:
        return float(default)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default=str(ROOT / "scenes" / "tank_current_scene.xml"), help="MuJoCo XML scene path")
    parser.add_argument("--profile-file", default=str(ROOT / "config" / "sim_profiles.json"), help="Simulation profile JSON")
    parser.add_argument("--profile", default="current")
    parser.add_argument("--water-surface-z", type=float, default=env_float("UUV_WATER_SURFACE_Z", 0.0))
    parser.add_argument(
        "--surface-depth-cm",
        type=float,
        default=env_float("SITL_SURFACE_DEPTH", -10.0),
        help="SITL SURFACE_DEPTH value in centimetres; sign is ignored for clearance.",
    )
    parser.add_argument(
        "--thruster-half-height-m",
        type=float,
        default=env_float("UUV_THRUSTER_IMMERSION_HALF_HEIGHT_M", 0.045),
    )
    parser.add_argument(
        "--minimum-bar30-depth-m",
        type=float,
        default=env_float("UUV_INITIAL_BAR30_MIN_DEPTH_M", 0.60),
        help="Minimum positive-down Bar30 start depth candidate.",
    )
    parser.add_argument(
        "--margin-m",
        type=float,
        default=env_float("UUV_INITIAL_SUBMERGED_MARGIN_M", 0.08),
        help="Extra depth margin used for fully-wet start calculation",
    )
    parser.add_argument("--depth", type=float, action="append", default=[], help="Additional base_link positive-down depths to audit")
    parser.add_argument("--output-dir", default=str(ROOT / "debug" / "physics_contract"), help="Directory for CSV/JSON audit outputs")
    parser.add_argument("--simulate-s", type=float, default=0.0, help="Run open-plant neutral-PWM MuJoCo dynamics for this duration at audited depths")
    parser.add_argument("--buoyancy-scale", type=float, default=None, help="Temporary audit-only override for sim_profile buoyancy_scale")
    parser.add_argument("--cob-x-offset", type=float, default=None, help="Temporary audit-only override for sim_profile cob_x_offset")
    parser.add_argument("--cob-z-offset", type=float, default=None, help="Temporary audit-only override for sim_profile cob_z_offset")
    parser.add_argument("--cob-torque-scale", type=float, default=None, help="Temporary audit-only override for sim_profile cob_torque_scale")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from physics_contract_runner import run_physics_contract_audit

    return run_physics_contract_audit(args)


if __name__ == "__main__":
    raise SystemExit(main())
