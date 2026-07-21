#!/usr/bin/env python3
"""Extract a MuJoCo initial-state contract from a real controller feedback CSV."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from real_start_baro import (
    AP_BARO_FRONTEND_PA_PER_M,
    AP_BARO_SITL_GROUND_PRESSURE_PA,
    baro_frontend_depth_m,
    baro_json_depth_for_frontend_match,
    infer_baro_real_ground_pressure,
    surface_pressure_for_sample,
)
from real_start_builder import build_state
from real_start_common import finite, truthy
from real_start_csv import pick_row, read_rows
from real_start_extractors import (
    angular_velocity_from_row,
    base_depth_from_row,
    base_xy_from_row,
    depth_from_row,
    rpy_from_row,
    velocity_from_row,
)
from real_start_geometry import mat_transpose_vec_mul, quat_xyzw_to_rotmat, quat_xyzw_to_rpy
from real_start_output import print_shell


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--format", choices=("json", "shell"), default="json")
    args = parser.parse_args()

    state = build_state(args.csv, args.start)
    if args.format == "shell":
        print_shell(state)
    else:
        print(json.dumps(state, indent=2, sort_keys=True))
    return 0


__all__ = [
    "AP_BARO_FRONTEND_PA_PER_M",
    "AP_BARO_SITL_GROUND_PRESSURE_PA",
    "angular_velocity_from_row",
    "baro_frontend_depth_m",
    "baro_json_depth_for_frontend_match",
    "base_depth_from_row",
    "base_xy_from_row",
    "build_state",
    "depth_from_row",
    "finite",
    "infer_baro_real_ground_pressure",
    "mat_transpose_vec_mul",
    "pick_row",
    "print_shell",
    "quat_xyzw_to_rotmat",
    "quat_xyzw_to_rpy",
    "read_rows",
    "rpy_from_row",
    "surface_pressure_for_sample",
    "truthy",
    "velocity_from_row",
]


if __name__ == "__main__":
    raise SystemExit(main())
