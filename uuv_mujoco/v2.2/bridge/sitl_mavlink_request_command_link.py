"""SERVO_OUTPUT_RAW request policy for the command MAVLink link."""

from __future__ import annotations

from .sitl_mavlink_request_targets import resolve_command_link_target


def request_command_link_servo_stream(transport, *, now_wall: float) -> None:
    if not transport._sitl_command_link_telemetry_enabled:
        return
    if transport._sitl_cmd_mav is None or transport._sitl_mavutil is None:
        return
    target = resolve_command_link_target(transport)
    if target is None:
        return
    target_sys, target_comp = target
    key = "command_link_servo_output_raw"
    transport._mavlink_interval_requester.request_servo_output_raw(
        key=key,
        mav=transport._sitl_cmd_mav,
        mavutil=transport._sitl_mavutil,
        target_sys=target_sys,
        target_comp=target_comp,
        now_wall=now_wall,
        requested_hz=transport._sitl_rcout_telemetry_hz,
        period_s=5.0,
    )
    transport._sitl_cmd_servo_last_req_wall = transport._mavlink_interval_requester.last_wall(key)


__all__ = ["request_command_link_servo_stream"]
