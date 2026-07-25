"""CSV row parser for native VISION_POSITION_DELTA replay events."""

from __future__ import annotations

import numpy as np

from bridge.sitl_replay_common import csv_float
from bridge.sitl_replay_row_vectors import row_vector3
from bridge.sitl_replay_types import NativeVisionDeltaEvent


def native_vpd_event_from_row(row: dict[str, str], *, real_start_s: float) -> NativeVisionDeltaEvent | None:
    t_real_s = csv_float(row, "t_s", np.nan)
    if not np.isfinite(t_real_s):
        return None
    return NativeVisionDeltaEvent(
        t_real_s=float(t_real_s),
        t_replay_s=float(t_real_s) - float(real_start_s),
        time_usec=int(csv_float(row, "time_usec", 0.0)),
        time_delta_usec=max(0, int(csv_float(row, "time_delta_usec", 0.0))),
        angle_delta=row_vector3(row, ("angle_delta_x", "angle_delta_y", "angle_delta_z"), (0.0, 0.0, 0.0)),
        position_delta=row_vector3(
            row,
            ("position_delta_x", "position_delta_y", "position_delta_z"),
            (0.0, 0.0, 0.0),
        ),
        confidence=float(csv_float(row, "confidence", 0.0)),
    )


__all__ = ["native_vpd_event_from_row"]
