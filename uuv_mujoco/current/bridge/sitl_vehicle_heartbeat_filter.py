"""Vehicle HEARTBEAT filtering for SITL MAVLink links."""

from __future__ import annotations

from .sitl_vehicle_heartbeat_target import heartbeat_autopilot_matches, heartbeat_source_matches_target


def _heartbeat_is_vehicle(self, msg) -> bool:
    if msg is None or self._sitl_mavutil is None:
        return False
    try:
        if msg.get_type() != "HEARTBEAT":
            return False
        return heartbeat_source_matches_target(self, msg) and heartbeat_autopilot_matches(self, msg)
    except Exception:
        return False


__all__ = ["_heartbeat_is_vehicle"]
