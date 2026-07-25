"""Position-target extraction for MAVROS raw local setpoints."""

from __future__ import annotations

import numpy as np


def position_target_ned(self, msg, tmask: int) -> tuple[np.ndarray, bool]:
    target_ned = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    has_pos = False
    pos = getattr(msg, "position", None)
    if pos is None:
        pos = getattr(msg, "position_", None)
    if pos is None:
        return target_ned, has_pos
    if not (tmask & self._POSITION_TARGET_TYPEMASK_X_IGNORE):
        target_ned[0] = float(getattr(pos, "x", 0.0))
        has_pos = True
    if not (tmask & self._POSITION_TARGET_TYPEMASK_Y_IGNORE):
        target_ned[1] = float(getattr(pos, "y", 0.0))
        has_pos = True
    if not (tmask & self._POSITION_TARGET_TYPEMASK_Z_IGNORE):
        target_ned[2] = float(getattr(pos, "z", 0.0))
        has_pos = True
    return target_ned, has_pos


__all__ = ["position_target_ned"]
