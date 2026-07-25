"""Low-level MAVLink GCS heartbeat sender."""

from __future__ import annotations

import time


def send_gcs_heartbeat_on_link(
    mav: object | None,
    mavutil: object,
    *,
    last_send_wall: float,
    force: bool = False,
    now_wall: float | None = None,
) -> tuple[bool, float]:
    if mav is None or mavutil is None:
        return False, float(last_send_wall)
    now = time.monotonic() if now_wall is None else float(now_wall)
    if not force and now - float(last_send_wall) < 1.0:
        return False, float(last_send_wall)
    try:
        mavlink_defs = mavutil.mavlink
        mav.mav.heartbeat_send(
            int(mavlink_defs.MAV_TYPE_GCS),
            int(mavlink_defs.MAV_AUTOPILOT_INVALID),
            0,
            0,
            int(mavlink_defs.MAV_STATE_ACTIVE),
        )
        return True, now
    except Exception:
        return False, float(last_send_wall)


__all__ = ["send_gcs_heartbeat_on_link"]
