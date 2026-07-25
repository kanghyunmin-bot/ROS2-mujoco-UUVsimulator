"""Receive helpers for the dedicated MAVLink command link."""

from __future__ import annotations

from sim.transport import AP_TELEMETRY_TYPES


def _recv_command_link_message(self):
    try:
        return self._sitl_cmd_mav.recv_match(
            type=["COMMAND_ACK", *AP_TELEMETRY_TYPES],
            blocking=False,
        )
    except Exception:
        return None


__all__ = ["_recv_command_link_message"]
