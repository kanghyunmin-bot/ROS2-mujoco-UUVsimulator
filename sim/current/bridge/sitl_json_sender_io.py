"""JSON encoding and receiver-state synchronization for SITL sensor sends."""

from __future__ import annotations

import json


def encode_sitl_json_payload(payload: dict[str, object]) -> bytes:
    # ArduSub-4.1.2's lightweight JSON parser expects vector values to start
    # immediately after the colon, e.g. "gyro":[...].
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def current_sitl_json_target(self):
    return self._sitl_client_addr if self._sitl_client_addr is not None else self.sitl_send_addr


def send_sitl_json_bytes(self, payload: dict[str, object], target):
    msg = encode_sitl_json_payload(payload)
    sent, target = self._json_servo_receiver.send_bytes(msg, target=target)
    self.sitl_sock = self._json_servo_receiver.socket
    self._sitl_send_counter = self._json_servo_receiver.send_counter
    self._sitl_send_target = self._json_servo_receiver.send_target
    return int(sent), target


__all__ = ["current_sitl_json_target", "encode_sitl_json_payload", "send_sitl_json_bytes"]
