"""DVL dead-reckoning compatibility message builder."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .ros2_dvl_header import stamp_header
from .ros2_math import set_first_attr, set_nested_xyz
from .sitl_math import quat_to_rpy


def _set_dvl_position_fields(msg: Any, position_flu: np.ndarray) -> None:
    set_nested_xyz(msg, "position", position_flu)
    set_nested_xyz(msg, "pose", position_flu)
    set_first_attr(msg, ("x", "position_x"), float(position_flu[0]))
    set_first_attr(msg, ("y", "position_y"), float(position_flu[1]))
    set_first_attr(msg, ("z", "position_z"), float(position_flu[2]))


def _set_dvl_attitude_fields(
    msg: Any,
    quat_ros: np.ndarray,
    attitude_rpy_deg: tuple[float, float, float] | None,
) -> None:
    if not (hasattr(msg, "roll") or hasattr(msg, "pitch") or hasattr(msg, "yaw")):
        return
    if attitude_rpy_deg is None:
        attitude_rpy_deg = tuple(
            math.degrees(value) for value in quat_to_rpy(quat_ros)
        )
    roll_deg, pitch_deg, yaw_deg = attitude_rpy_deg
    set_first_attr(msg, ("roll",), float(roll_deg))
    set_first_attr(msg, ("pitch",), float(pitch_deg))
    set_first_attr(msg, ("yaw",), float(yaw_deg))


def build_dvldr_msg(
    dvldr_msg_type: type | None,
    stamp: Any,
    position_flu: np.ndarray,
    quat_ros: np.ndarray,
    *,
    attitude_rpy_deg: tuple[float, float, float] | None = None,
    position_std_m: float = 0.0,
    report_time_s: float = 0.0,
) -> Any | None:
    if dvldr_msg_type is None:
        return None
    msg = dvldr_msg_type()
    stamp_header(msg, stamp, "dvl_link")
    _set_dvl_position_fields(msg, position_flu)
    _set_dvl_attitude_fields(msg, quat_ros, attitude_rpy_deg)
    set_first_attr(msg, ("time",), float(report_time_s))
    set_first_attr(msg, ("pos_std",), float(max(position_std_m, 0.0)))
    set_first_attr(msg, ("type",), "position_local")
    set_first_attr(msg, ("status",), 0)
    set_first_attr(msg, ("format",), "json_v3")
    return msg


__all__ = ["build_dvldr_msg"]
