"""Timestamp helpers for MAVROS-compatible RCOut telemetry."""

from __future__ import annotations

import math


def sensor_replay_real_stamp(bridge, stamp_type):
    if bridge._mavros_rc_out_header_stamp_source != "sensor_replay_real":
        return None
    try:
        status_getter = getattr(bridge._sitl_transport, "sensor_replay_status", None)
        status = status_getter() if callable(status_getter) else {}
        replay_real_t = status.get("current_real_t_s")
        if replay_real_t is None:
            return None
        replay_real_t = max(0.0, float(replay_real_t))
        sec = int(math.floor(replay_real_t))
        nanosec = int(round((replay_real_t - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp_msg = stamp_type()
        stamp_msg.sec = int(sec)
        stamp_msg.nanosec = int(nanosec)
        return stamp_msg
    except Exception:
        return None


__all__ = ["sensor_replay_real_stamp"]
