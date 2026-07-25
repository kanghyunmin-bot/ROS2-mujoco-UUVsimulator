"""MAVLink peer state helpers for SitlTransport."""

from __future__ import annotations

import time


def recent_vehicle_heartbeat(owner, mav) -> bool:
    now = time.monotonic()
    if mav is owner._sitl_cmd_mav and owner._sitl_cmd_mav_last_hb_wall > 0.0:
        return now - owner._sitl_cmd_mav_last_hb_wall < 2.0
    if mav is owner._sitl_mav and owner._sitl_mav_last_hb_wall > 0.0:
        return now - owner._sitl_mav_last_hb_wall < 2.0
    return False


def endpoint_for_mav(owner, mav) -> str:
    if mav is owner._sitl_cmd_mav:
        return owner._sitl_cmd_mavlink_endpoint
    return owner._sitl_mavlink_endpoint


def mavlink_has_udp_peer(mav) -> bool:
    return bool(getattr(mav, "clients", None))


def mavlink_requires_peer_wait(owner, mav) -> bool:
    endpoint = endpoint_for_mav(owner, mav)
    return str(endpoint).strip().lower().startswith("udpin:") and not mavlink_has_udp_peer(mav)


__all__ = [
    "endpoint_for_mav",
    "mavlink_has_udp_peer",
    "mavlink_requires_peer_wait",
    "recent_vehicle_heartbeat",
]
