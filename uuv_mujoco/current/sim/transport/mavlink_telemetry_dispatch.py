"""MAVLink message dispatch for passive telemetry observation."""

from __future__ import annotations

from typing import Callable

from .mavlink_telemetry import message_type
from .mavlink_telemetry_handlers import handle_target_mavlink_telemetry
from .mavlink_telemetry_storage import store_heartbeat


def observe_mavlink_telemetry(
    msg: object,
    now_wall: float,
    *,
    status_data: dict[str, object],
    source_matches_target: Callable[[object], bool],
) -> None:
    if msg is None:
        return
    msg_type = message_type(msg)
    if msg_type == "BAD_DATA":
        return
    try:
        data = msg.to_dict()
    except Exception:
        data = {}
    if not source_matches_target(msg):
        return
    status_data["last_msg_type"] = msg_type
    status_data["last_msg_wall_s"] = float(now_wall)

    if msg_type == "HEARTBEAT":
        store_heartbeat(status_data, msg, data, now_wall)
        return

    handle_target_mavlink_telemetry(status_data, msg_type, data, now_wall)


__all__ = ["observe_mavlink_telemetry"]
