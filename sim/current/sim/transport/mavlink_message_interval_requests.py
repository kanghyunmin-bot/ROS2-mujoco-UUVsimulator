"""Throttled MAVLink message-interval request bodies."""

from __future__ import annotations

from .mavlink_message_constants import DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS
from .mavlink_message_interval_send import (
    get_mavlink_defs,
    request_message_interval_by_constant,
    request_message_intervals_by_constants,
)
from .mavlink_message_interval_state import mark_if_sent, request_due


def request_servo_output_raw_interval(
    last_request_wall_by_key: dict[str, float],
    *,
    key: str,
    mav: object | None,
    mavutil: object | None,
    target_sys: int,
    target_comp: int,
    now_wall: float,
    requested_hz: float,
    period_s: float,
) -> bool:
    if not request_due(last_request_wall_by_key, key, now_wall, period_s):
        return False
    sent = request_message_interval_by_constant(
        mav=mav,
        mavlink_defs=get_mavlink_defs(mavutil),
        constant_name="MAVLINK_MSG_ID_SERVO_OUTPUT_RAW",
        target_sys=target_sys,
        target_comp=target_comp,
        requested_hz=requested_hz,
    )
    return mark_if_sent(last_request_wall_by_key, key, now_wall, sent)


def request_named_message_intervals(
    last_request_wall_by_key: dict[str, float],
    *,
    key: str,
    mav: object | None,
    mavutil: object | None,
    target_sys: int,
    target_comp: int,
    now_wall: float,
    requested_hz: float,
    period_s: float,
    message_constant_names: tuple[str, ...] = DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS,
) -> bool:
    if not request_due(last_request_wall_by_key, key, now_wall, period_s):
        return False
    sent_any = request_message_intervals_by_constants(
        mav=mav,
        mavlink_defs=get_mavlink_defs(mavutil),
        message_constant_names=message_constant_names,
        target_sys=target_sys,
        target_comp=target_comp,
        requested_hz=requested_hz,
    )
    return mark_if_sent(last_request_wall_by_key, key, now_wall, sent_any)


__all__ = [
    "DEFAULT_AP_TELEMETRY_MESSAGE_CONSTANTS",
    "request_named_message_intervals",
    "request_servo_output_raw_interval",
]
