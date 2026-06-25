"""Top-level SITL command, MAVLink, JSON-servo, and keepalive polling loop."""

from __future__ import annotations

import time


def poll_servo(self) -> None:
    now_wall = time.monotonic()
    if now_wall >= self._sitl_next_command_poll_wall:
        self._sitl_next_command_poll_wall = now_wall + 1.0 / max(self._sitl_command_poll_hz, 1.0)
        self._poll_command_mavlink()
    if now_wall >= self._sitl_next_mavlink_poll_wall:
        self._sitl_next_mavlink_poll_wall = now_wall + 1.0 / max(self._sitl_mavlink_poll_hz, 1.0)
        self._send_gcs_heartbeat(mav=self._sitl_mav)
        self._poll_servo_mavlink()
    self._poll_servo_endpoint()
    service_wall = time.monotonic()
    self._service_auto_ready_sequence(service_wall)
    self._service_plant_replay_timeout(service_wall)
    self._send_neutral_rc_keepalive(service_wall)


__all__ = ["poll_servo"]
