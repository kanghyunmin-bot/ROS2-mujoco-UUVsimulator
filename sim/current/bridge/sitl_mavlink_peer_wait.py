"""MAVLink UDP peer wait loop for SitlTransport."""

from __future__ import annotations

import time

from bridge.sitl_mavlink_peer_state import mavlink_has_udp_peer


def wait_for_mavlink_peer(owner, mav, *, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        msg = recv_peer_heartbeat(mav)
        if msg is not None and owner._heartbeat_is_vehicle(msg):
            owner._update_vehicle_heartbeat(msg, command_link=(mav is owner._sitl_cmd_mav))
            return True
        if mavlink_has_udp_peer(mav):
            return True
        time.sleep(0.02)
    return mavlink_has_udp_peer(mav)


def recv_peer_heartbeat(mav):
    try:
        return mav.recv_match(type=["HEARTBEAT"], blocking=False)
    except Exception:
        return None


__all__ = ["recv_peer_heartbeat", "wait_for_mavlink_peer"]
