"""JSON servo endpoint packet polling for SitlTransport."""

from __future__ import annotations

import time

from .sitl_json_servo_packet_state import (
    json_servo_packet_can_drive_plant,
    record_json_servo_client,
    record_json_servo_frame_state,
)
from .sitl_json_servo_warnings import warn_json_servo_client_status


def _poll_servo_endpoint(self) -> None:
    self.sitl_sock = self._json_servo_receiver.socket
    if not self.sitl_sock:
        return

    now_wall = time.monotonic()
    latest_pwm_values: list[int] | None = None
    for servo_packet, addr in self._json_servo_receiver.receive_packets(self._sitl_json_poll_budget):
        record_json_servo_frame_state(self, servo_packet)
        record_json_servo_client(self, addr, now_wall)
        if not json_servo_packet_can_drive_plant(self, now_wall):
            continue
        latest_pwm_values = servo_packet.pwm_values
        self._send_immediate_sensor_replay_reply(now_wall, int(servo_packet.frame_count))

    if latest_pwm_values is not None:
        self._handle_pwm_values(latest_pwm_values, now_wall, source="json")
    warn_json_servo_client_status(self, now_wall)


__all__ = ["_poll_servo_endpoint"]
