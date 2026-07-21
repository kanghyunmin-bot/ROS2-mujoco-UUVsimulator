"""Low-level MAVLink SET_MESSAGE_INTERVAL send helpers."""

from __future__ import annotations


def get_mavlink_defs(mavutil: object | None) -> object | None:
    return getattr(mavutil, "mavlink", None)


def resolve_message_id(mavlink_defs: object | None, constant_name: str) -> int | None:
    message_id = getattr(mavlink_defs, str(constant_name), None)
    return None if message_id is None else int(message_id)


def requested_interval_us(requested_hz: float) -> float:
    return float(max(1.0, 1.0e6 / max(float(requested_hz), 0.5)))


def send_message_interval(
    *,
    mav: object | None,
    mavlink_defs: object | None,
    target_sys: int,
    target_comp: int,
    message_id: int,
    requested_hz: float,
) -> bool:
    if mav is None or mavlink_defs is None:
        return False
    try:
        mav.mav.command_long_send(
            int(target_sys),
            int(target_comp),
            int(mavlink_defs.MAV_CMD_SET_MESSAGE_INTERVAL),
            0,
            float(message_id),
            requested_interval_us(requested_hz),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return True
    except Exception:
        return False


def request_message_interval_by_constant(
    *,
    mav: object | None,
    mavlink_defs: object | None,
    constant_name: str,
    target_sys: int,
    target_comp: int,
    requested_hz: float,
) -> bool:
    message_id = resolve_message_id(mavlink_defs, constant_name)
    if message_id is None:
        return False
    return send_message_interval(
        mav=mav,
        mavlink_defs=mavlink_defs,
        target_sys=target_sys,
        target_comp=target_comp,
        message_id=message_id,
        requested_hz=requested_hz,
    )


def request_message_intervals_by_constants(
    *,
    mav: object | None,
    mavlink_defs: object | None,
    message_constant_names: tuple[str, ...],
    target_sys: int,
    target_comp: int,
    requested_hz: float,
) -> bool:
    sent_any = False
    for constant_name in message_constant_names:
        sent_any = request_message_interval_by_constant(
            mav=mav,
            mavlink_defs=mavlink_defs,
            constant_name=constant_name,
            target_sys=target_sys,
            target_comp=target_comp,
            requested_hz=requested_hz,
        ) or sent_any
    return sent_any


__all__ = [
    "get_mavlink_defs",
    "request_message_interval_by_constant",
    "request_message_intervals_by_constants",
    "requested_interval_us",
    "resolve_message_id",
    "send_message_interval",
]
