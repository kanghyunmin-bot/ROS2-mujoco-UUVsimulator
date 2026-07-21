"""MAVLink receive helper for the SERVO_OUTPUT_RAW link."""

from __future__ import annotations

from sim.transport import AP_TELEMETRY_TYPES


def recv_servo_link_message(transport):
    try:
        return transport._sitl_mav.recv_match(
            type=["COMMAND_ACK", *AP_TELEMETRY_TYPES],
            blocking=False,
        )
    except Exception:
        return None


__all__ = ["recv_servo_link_message"]
