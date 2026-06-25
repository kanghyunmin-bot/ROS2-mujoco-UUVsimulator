"""CSV row parser for full-runtime SITL sensor replay frames."""

from __future__ import annotations

import numpy as np

from bridge.sitl_replay_common import csv_float, normalize_quat_wxyz, pressure_abs_from_depth_m
from bridge.sitl_replay_row_vectors import row_vector3
from bridge.sitl_replay_types import SensorReplayFrame


def sensor_frame_from_row(
    row: dict[str, str],
    *,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
    home_alt_m: float,
) -> SensorReplayFrame | None:
    t_s = csv_float(row, "t_replay_s", csv_float(row, "t_s", np.nan))
    if not np.isfinite(t_s):
        return None
    depth_m = csv_float(row, "depth_m", 0.0)
    extnav_pos = row_vector3(row, ("extnav_pos_n", "extnav_pos_e", "extnav_pos_d"), (np.nan, np.nan, np.nan))
    return SensorReplayFrame(
        t_s=float(t_s),
        gyro_frd=row_vector3(row, ("gyro_x", "gyro_y", "gyro_z"), (0.0, 0.0, 0.0)),
        accel_frd=row_vector3(row, ("accel_x", "accel_y", "accel_z"), (0.0, 0.0, -gravity)),
        quat_ned_frd=normalize_quat_wxyz(
            np.array(
                [
                    csv_float(row, "quat_w", 1.0),
                    csv_float(row, "quat_x", 0.0),
                    csv_float(row, "quat_y", 0.0),
                    csv_float(row, "quat_z", 0.0),
                ],
                dtype=np.float64,
            )
        ),
        depth_m=float(depth_m),
        pressure_pa=pressure_abs_from_depth_m(depth_m, surface_pressure_pa, water_density, gravity),
        pos_ned=row_vector3(row, ("pos_n", "pos_e", "pos_d"), (0.0, 0.0, depth_m)),
        vel_ned=row_vector3(row, ("vel_n", "vel_e", "vel_d"), (0.0, 0.0, 0.0)),
        alt_m=csv_float(row, "altitude_m", home_alt_m - depth_m),
        extnav_pos_ned=extnav_pos if np.all(np.isfinite(extnav_pos)) else None,
    )


__all__ = ["sensor_frame_from_row"]
