"""Low-level activity predicates for SITL MAVLink command links."""

from __future__ import annotations


COMMAND_LINK_STALE_S = 3.0


def recent_wall_activity(timestamp: float, now: float, *, max_age_s: float = COMMAND_LINK_STALE_S) -> bool:
    return float(timestamp) > 0.0 and float(now) - float(timestamp) < float(max_age_s)


def mavlink_object_has_clients(mav: object) -> bool:
    try:
        return bool(getattr(mav, "clients", None))
    except Exception:
        return False


def any_mavlink_object_has_clients(mavs: tuple[object | None, ...]) -> bool:
    return any(mavlink_object_has_clients(mav) for mav in mavs if mav is not None)


def any_heartbeat_seen(*heartbeats: object | None) -> bool:
    return any(heartbeat is not None for heartbeat in heartbeats)


__all__ = [
    "COMMAND_LINK_STALE_S",
    "any_heartbeat_seen",
    "any_mavlink_object_has_clients",
    "mavlink_object_has_clients",
    "recent_wall_activity",
]
