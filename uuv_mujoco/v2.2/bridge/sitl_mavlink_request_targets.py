"""Target and timing helpers for SITL MAVLink stream requests."""

from __future__ import annotations


def servo_stream_request_period(now_wall: float, *, last_msg_wall: float, servo_hz: float) -> float:
    stream_fresh = (
        float(last_msg_wall) > 0.0
        and (float(now_wall) - float(last_msg_wall)) <= max(2.0, 4.0 / max(float(servo_hz), 1.0))
    )
    return 12.0 if stream_fresh else 2.0


def resolve_servo_link_target(transport) -> tuple[int, int] | None:
    if transport._sitl_mavlink_target_sysid > 0:
        return int(transport._sitl_mavlink_target_sysid), int(transport._sitl_mavlink_target_compid)
    if transport._sitl_mav_hb is not None:
        return int(transport._sitl_mav_hb.get_srcSystem()), int(transport._sitl_mav_hb.get_srcComponent())
    return None


def resolve_command_link_target(transport) -> tuple[int, int] | None:
    return transport._resolve_mav_target(transport._sitl_cmd_mav)


__all__ = [
    "resolve_command_link_target",
    "resolve_servo_link_target",
    "servo_stream_request_period",
]
