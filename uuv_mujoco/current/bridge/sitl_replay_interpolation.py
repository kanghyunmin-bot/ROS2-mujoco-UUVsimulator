"""Interpolation facade for full-runtime SITL sensor replay."""

from __future__ import annotations

from bridge.sitl_replay_interp_frame import build_interpolated_sensor_frame
from bridge.sitl_replay_interp_index import boundary_sensor_frame, bracket_sensor_frames
from bridge.sitl_replay_types import SensorReplayFrame


def interpolate_sensor_replay_frame(
    frames: list[SensorReplayFrame],
    *,
    t_s: float,
    start_index: int,
    surface_pressure_pa: float,
    water_density: float,
    gravity: float,
) -> tuple[SensorReplayFrame, int | None]:
    """Interpolate a replay frame without owning replay clock state."""
    if not frames:
        raise ValueError("interpolate_sensor_replay_frame requires at least one frame")
    boundary = boundary_sensor_frame(frames, t_s)
    if boundary is not None:
        return boundary, None
    a, b, idx, alpha = bracket_sensor_frames(frames, t_s=t_s, start_index=start_index)
    return (
        build_interpolated_sensor_frame(
            a,
            b,
            t_s=t_s,
            alpha=alpha,
            surface_pressure_pa=surface_pressure_pa,
            water_density=water_density,
            gravity=gravity,
        ),
        idx,
    )


__all__ = ["interpolate_sensor_replay_frame"]
