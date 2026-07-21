"""MAVLink command-link polling loop for SitlTransport."""

from __future__ import annotations

import time

from .sitl_mavlink_command_handlers import _handle_command_link_message
from .sitl_mavlink_command_receive import _recv_command_link_message


def _poll_command_mavlink(self) -> None:
    self._ensure_command_mavlink_connected()
    if self._sitl_cmd_mav is None:
        return
    now_wall = time.monotonic()
    self._send_gcs_heartbeat(mav=self._sitl_cmd_mav)
    for _ in range(self._sitl_command_poll_budget):
        msg = _recv_command_link_message(self)
        if msg is None:
            break
        _handle_command_link_message(self, msg, now_wall)
    if not self._arm_mode_command_pending():
        self._request_command_servo_telemetry_stream(now_wall)
        self._request_command_ap_telemetry_stream(now_wall)


__all__ = ["_handle_command_link_message", "_poll_command_mavlink"]
