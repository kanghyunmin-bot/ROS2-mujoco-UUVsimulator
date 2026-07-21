"""Servo-link HEARTBEAT handling for SitlTransport."""

from __future__ import annotations

from .sitl_mavlink_servo_heartbeat_target import _ardupilotmega_value, _servo_heartbeat_target_ok


def _handle_servo_link_heartbeat(self, msg, now_wall: float) -> None:
    src_sys = int(msg.get_srcSystem())
    src_comp = int(msg.get_srcComponent())
    try:
        ap = int(getattr(msg, "autopilot", -1))
    except Exception:
        ap = -1
    autopilot_mega = _ardupilotmega_value(self)
    target_sys = self._sitl_mavlink_target_sysid
    target_comp = self._sitl_mavlink_target_compid
    target_must_match = target_sys > 0 or target_comp > 0
    if target_must_match and ap != autopilot_mega:
        return
    if not _servo_heartbeat_target_ok(self, src_sys, src_comp, target_sys, target_comp, now_wall):
        return
    if (not target_must_match) and (ap != autopilot_mega):
        return
    self._store_ap_mavlink_telemetry(msg, now_wall)
    if target_must_match or ap == autopilot_mega:
        self._update_vehicle_heartbeat(msg, command_link=False)
        self._request_sitl_mavlink_servo_stream()
        self._request_sitl_mavlink_ap_telemetry_stream()


__all__ = [
    "_ardupilotmega_value",
    "_handle_servo_link_heartbeat",
    "_servo_heartbeat_target_ok",
]
