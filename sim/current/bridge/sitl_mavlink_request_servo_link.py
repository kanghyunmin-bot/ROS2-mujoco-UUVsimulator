"""SERVO_OUTPUT_RAW request policy for the servo MAVLink link."""

from __future__ import annotations

from .sitl_mavlink_request_targets import resolve_servo_link_target, servo_stream_request_period


def request_servo_link_servo_stream(transport, *, now_wall: float) -> None:
    if not transport._sitl_request_servo_interval:
        return
    if transport._sitl_mav is None or transport._sitl_mavutil is None:
        return
    target = resolve_servo_link_target(transport)
    if target is None:
        return
    target_sys, target_comp = target
    period_s = servo_stream_request_period(
        now_wall,
        last_msg_wall=transport._sitl_mav_last_msg_wall,
        servo_hz=transport._sitl_mavlink_servo_hz,
    )
    key = "servo_link_servo_output_raw"
    transport._mavlink_interval_requester.request_servo_output_raw(
        key=key,
        mav=transport._sitl_mav,
        mavutil=transport._sitl_mavutil,
        target_sys=target_sys,
        target_comp=target_comp,
        now_wall=now_wall,
        requested_hz=transport._sitl_mavlink_servo_hz,
        period_s=period_s,
    )
    transport._sitl_mav_last_req_wall = transport._mavlink_interval_requester.last_wall(key)


__all__ = ["request_servo_link_servo_stream"]
