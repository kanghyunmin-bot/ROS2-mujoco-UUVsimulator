"""Build real-start state dictionaries from controller feedback rows."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from real_start_common import finite, truthy
from real_start_csv import pick_row, read_rows
from real_start_extractors import (
    angular_velocity_from_row,
    base_depth_from_row,
    base_xy_from_row,
    depth_from_row,
    rpy_from_row,
)
from real_start_velocity_policy import velocity_from_rows
from real_start_pressure_state import build_pressure_state
from real_start_rc_state import rc_override_ch1_8_from_row


def build_state(path: Path, start_s: float) -> dict[str, Any]:
    rows, time_col = read_rows(path)
    row = pick_row(rows, time_col, start_s)
    velocity_source_policy = os.environ.get("UUV_REAL_START_VELOCITY_SOURCE", "local_pose")
    depth_m, depth_source = depth_from_row(row)
    base_depth_m, base_depth_source = base_depth_from_row(row)
    base_x_m, base_y_m, base_xy_source = base_xy_from_row(row)
    roll, pitch, yaw, attitude_source = rpy_from_row(row)
    vx, vy, vz, velocity_source = velocity_from_rows(rows, time_col, row, velocity_source_policy)
    wx, wy, wz, angular_velocity_source = angular_velocity_from_row(row)
    pressure_state = build_pressure_state(
        rows=rows,
        time_col=time_col,
        row=row,
        depth_m=depth_m,
        depth_source=depth_source,
    )
    return {
        "csv": str(path),
        "requested_start_s": float(start_s),
        "source_time_column": time_col,
        "source_t_s": finite(row.get(time_col)),
        "depth_m": depth_m,
        "depth_source": depth_source,
        "base_depth_m": base_depth_m,
        "base_depth_source": base_depth_source,
        "base_x_m": base_x_m,
        "base_y_m": base_y_m,
        "base_xy_source": base_xy_source,
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "attitude_source": attitude_source,
        "body_vx_mps": vx,
        "body_vy_mps": vy,
        "body_vz_mps": vz,
        "velocity_source": velocity_source,
        "velocity_source_policy": velocity_source_policy,
        "body_wx_radps": wx,
        "body_wy_radps": wy,
        "body_wz_radps": wz,
        "angular_velocity_source": angular_velocity_source,
        **pressure_state,
        "mode": str(row.get("mode", "")).strip(),
        "armed": truthy(row.get("armed", "0")),
        "rc_override_ch1_8": rc_override_ch1_8_from_row(row),
    }
