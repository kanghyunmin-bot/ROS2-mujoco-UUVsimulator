"""Range/altitude ROS2 message builders."""

from __future__ import annotations

from typing import Any


def build_range_msg(range_type: type, stamp: Any, distance_m: float, frame_id: str = "dvl_link") -> Any:
    msg = range_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.radiation_type = range_type.ULTRASOUND
    msg.field_of_view = 0.25
    msg.min_range = 0.05
    msg.max_range = 30.0
    msg.range = float("inf") if distance_m < 0.0 else float(distance_m)
    return msg


__all__ = ["build_range_msg"]
