"""MAVLink UDP peer discovery helpers for SitlTransport."""

from __future__ import annotations

from bridge.sitl_mavlink_peer_state import (
    mavlink_has_udp_peer,
    mavlink_requires_peer_wait,
    recent_vehicle_heartbeat,
)
from bridge.sitl_mavlink_peer_wait import recv_peer_heartbeat, wait_for_mavlink_peer


def _ensure_mavlink_peer(self, mav, *, timeout_s: float = 1.5) -> bool:
    """For udpin links, wait until pymavlink has seen ArduSub's UDP peer."""
    if mav is None:
        return False
    if recent_vehicle_heartbeat(self, mav):
        return True
    if not mavlink_requires_peer_wait(self, mav):
        return True
    return wait_for_mavlink_peer(self, mav, timeout_s=timeout_s)


__all__ = [
    "_ensure_mavlink_peer",
    "mavlink_has_udp_peer",
    "mavlink_requires_peer_wait",
    "recent_vehicle_heartbeat",
    "recv_peer_heartbeat",
    "wait_for_mavlink_peer",
]
