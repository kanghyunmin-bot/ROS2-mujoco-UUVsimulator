"""Profile and hydrodynamics CLI options for the UUV MuJoCo runner."""

from __future__ import annotations

import argparse
from pathlib import Path


def add_profile_args(
    parser: argparse.ArgumentParser,
    *,
    scenes_dir: Path,
    profile_path: Path,
    thruster_perf_path: Path,
) -> None:
    parser.add_argument(
        "--scene",
        type=str,
        default=str(scenes_dir / "tank_current_scene.xml"),
        help="MJCF scene path (default: scenes/tank_current_scene.xml)",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="current",
        help="Simulation profile name (see --list-profiles)",
    )
    parser.add_argument(
        "--profile-file",
        type=str,
        default=str(profile_path),
        help="Simulation profile JSON path",
    )
    parser.add_argument(
        "--thruster-perf-file",
        type=str,
        default=str(thruster_perf_path),
        help="PWM-thrust performance curve JSON file",
    )
    parser.add_argument(
        "--thruster-voltage",
        type=float,
        default=None,
        help="Select nearest thrust curve voltage from performance file (ex. 10,12,14,16,18,20). If omitted, use profile value.",
    )
    parser.add_argument(
        "--buoyancy-scale",
        type=float,
        default=None,
        help="Override profile buoyancy scale at runtime (ex. 0.98 for weaker buoyancy).",
    )
    parser.add_argument(
        "--disable-thruster-perf",
        action="store_true",
        help="Force linear thruster mapping and ignore performance curve JSON",
    )
    parser.add_argument(
        "--thruster-perf-direct",
        action="store_true",
        help=(
            "When the performance curve is active, map raw normalized PWM directly "
            "to force and bypass profile command shaping/gain scaling."
        ),
    )
    parser.add_argument(
        "--fluid-model",
        type=str,
        default="current",
        help=(
            "Hydrodynamic model selection. current/ellipsoid uses MuJoCo geom "
            "fluidcoef; legacy/custom uses the Python 6DOF damping contract "
            "and disables MuJoCo built-in fluid to avoid double counting."
        ),
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help="Print available simulation profiles and exit",
    )
