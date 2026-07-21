"""MAVROS VFR HUD compatibility message builder."""

from __future__ import annotations

from typing import Any


def build_vfr_hud_msg(vfr_hud_type: type, stamp: Any, depth_m: float) -> Any:
    msg = vfr_hud_type()
    if hasattr(msg, "header"):
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
    if hasattr(msg, "airspeed"):
        msg.airspeed = 0.0
    if hasattr(msg, "groundspeed"):
        msg.groundspeed = 0.0
    if hasattr(msg, "heading"):
        msg.heading = 0
    if hasattr(msg, "throttle"):
        msg.throttle = 0.0
    if hasattr(msg, "altitude"):
        # The real ArduSub/MAVROS bag exposes a small meter-scale value here.
        msg.altitude = float(depth_m)
    if hasattr(msg, "climb"):
        msg.climb = 0.0
    return msg


__all__ = ["build_vfr_hud_msg"]
