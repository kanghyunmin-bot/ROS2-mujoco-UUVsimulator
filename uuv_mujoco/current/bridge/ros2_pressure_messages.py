"""Fluid pressure ROS2 message builders."""

from __future__ import annotations

from typing import Any


def build_pressure_msg(
    fluid_pressure_type: type,
    stamp: Any,
    pressure_pa: float,
    frame_id: str = "base_link",
    *,
    variance_pa2: float = 0.0,
) -> Any:
    msg = fluid_pressure_type()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.fluid_pressure = float(pressure_pa)
    msg.variance = float(variance_pa2)
    return msg


__all__ = ["build_pressure_msg"]
