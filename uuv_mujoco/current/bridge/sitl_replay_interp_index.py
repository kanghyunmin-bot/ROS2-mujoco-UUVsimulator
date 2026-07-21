"""Index search helpers for SITL sensor replay interpolation."""

from __future__ import annotations

import numpy as np

from bridge.sitl_replay_types import SensorReplayFrame


def boundary_sensor_frame(frames: list[SensorReplayFrame], t_s: float) -> SensorReplayFrame | None:
    if t_s <= frames[0].t_s:
        return frames[0]
    if t_s >= frames[-1].t_s:
        return frames[-1]
    return None


def bracket_sensor_frames(
    frames: list[SensorReplayFrame],
    *,
    t_s: float,
    start_index: int,
) -> tuple[SensorReplayFrame, SensorReplayFrame, int, float]:
    idx = int(np.clip(start_index, 0, len(frames) - 2))
    while idx + 1 < len(frames) and frames[idx + 1].t_s < t_s:
        idx += 1
    while idx > 0 and frames[idx].t_s > t_s:
        idx -= 1
    a = frames[idx]
    b = frames[idx + 1]
    dt = max(1.0e-9, b.t_s - a.t_s)
    alpha = float(np.clip((t_s - a.t_s) / dt, 0.0, 1.0))
    return a, b, idx, alpha


__all__ = ["boundary_sensor_frame", "bracket_sensor_frames"]
