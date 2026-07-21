"""Frame assembly for SITL sensor replay interpolation."""

from __future__ import annotations

from bridge.sitl_replay_common import normalize_quat_wxyz, pressure_abs_from_depth_m
from bridge.sitl_replay_interp_math import lerp_scalar, lerp_vec, shortest_quat_pair
from bridge.sitl_replay_types import SensorReplayFrame


def build_interpolated_sensor_frame(
    a: SensorReplayFrame,
    b: SensorReplayFrame,
    *,
    t_s: float,
    alpha: float,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
) -> SensorReplayFrame:
    quat_a, quat_b = shortest_quat_pair(a.quat_ned_frd, b.quat_ned_frd)
    extnav_pos = None
    if a.extnav_pos_ned is not None and b.extnav_pos_ned is not None:
        extnav_pos = lerp_vec(a.extnav_pos_ned, b.extnav_pos_ned, alpha)
    depth_m = lerp_scalar(a.depth_m, b.depth_m, alpha)
    return SensorReplayFrame(
        t_s=float(t_s),
        gyro_frd=lerp_vec(a.gyro_frd, b.gyro_frd, alpha),
        accel_frd=lerp_vec(a.accel_frd, b.accel_frd, alpha),
        quat_ned_frd=normalize_quat_wxyz(lerp_vec(quat_a, quat_b, alpha)),
        depth_m=depth_m,
        pressure_pa=pressure_abs_from_depth_m(depth_m, surface_pressure_pa, water_density, gravity),
        pos_ned=lerp_vec(a.pos_ned, b.pos_ned, alpha),
        vel_ned=lerp_vec(a.vel_ned, b.vel_ned, alpha),
        alt_m=lerp_scalar(a.alt_m, b.alt_m, alpha),
        extnav_pos_ned=extnav_pos,
    )


__all__ = ["build_interpolated_sensor_frame"]
