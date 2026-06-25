"""DVL velocity/range compatibility message builder."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ros2_dvl_header import stamp_header
from .ros2_math import set_first_attr, set_nested_xyz


def _set_dvl_velocity_fields(msg: Any, vel_dvl_frd: np.ndarray) -> None:
    set_nested_xyz(msg, "velocity", vel_dvl_frd)
    set_nested_xyz(msg, "vel", vel_dvl_frd)
    set_first_attr(msg, ("velocity_x", "vx", "surge_velocity"), float(vel_dvl_frd[0]))
    set_first_attr(msg, ("velocity_y", "vy", "sway_velocity"), float(vel_dvl_frd[1]))
    set_first_attr(msg, ("velocity_z", "vz", "heave_velocity"), float(vel_dvl_frd[2]))


def _set_dvl_altitude_fields(msg: Any, altitude_m: float | None) -> None:
    if altitude_m is not None and np.isfinite(altitude_m):
        set_first_attr(msg, ("altitude", "range", "height"), float(altitude_m))


def build_dvl_msg(
    dvl_msg_type: type | None,
    stamp: Any,
    vel_dvl_frd: np.ndarray | None,
    altitude_m: float | None,
) -> Any | None:
    if dvl_msg_type is None:
        return None
    msg = dvl_msg_type()
    stamp_header(msg, stamp, "dvl_link")
    if vel_dvl_frd is not None:
        _set_dvl_velocity_fields(msg, vel_dvl_frd)
    _set_dvl_altitude_fields(msg, altitude_m)
    set_first_attr(msg, ("valid", "is_valid", "bottom_lock"), True)
    return msg


__all__ = ["build_dvl_msg"]
