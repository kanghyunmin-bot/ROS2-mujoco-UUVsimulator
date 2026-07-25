"""Dedicated SITL MAVLink command-link connection setup and reconnect."""

from __future__ import annotations

import time


def _command_mavlink_disabled(self) -> bool:
    self._sitl_command_link.set_endpoint(self._sitl_cmd_mavlink_endpoint)
    return self._sitl_command_link.disabled


def _connect_sitl_command_mavlink(self, mavutil) -> None:
    endpoint = self._sitl_cmd_mavlink_endpoint
    self._sitl_command_link.set_endpoint(endpoint)
    if self._command_mavlink_disabled():
        log_command_link_uses_servo_link(endpoint)
        return
    try:
        self._sitl_cmd_mav = self._sitl_command_link.connect(mavutil)
        print(
            f"[sitl_transport] SITL MAVLink command output enabled: endpoint={endpoint}",
            flush=True,
        )
    except Exception as exc:
        self._sitl_cmd_mav = None
        self._sitl_command_link.set_connection(None)
        print(f"[sitl_transport] SITL MAVLink command output unavailable: {exc}", flush=True)


def log_command_link_uses_servo_link(endpoint: str | None) -> None:
    if endpoint:
        print("[sitl_transport] SITL MAVLink command output using servo link", flush=True)


def _ensure_command_mavlink_connected(self) -> None:
    if command_link_connect_not_needed(self):
        return
    now_wall = time.monotonic()
    if not self._sitl_command_link.should_attempt_reconnect(now_wall, min_interval_s=2.0):
        return
    self._sitl_cmd_mav_last_connect_attempt_wall = self._sitl_command_link.last_connect_attempt_wall
    self._connect_sitl_command_mavlink(self._sitl_mavutil)


def command_link_connect_not_needed(self) -> bool:
    return (
        self._sitl_cmd_mav is not None
        or self._sitl_mavutil is None
        or self._command_mavlink_disabled()
    )


__all__ = [
    "_command_mavlink_disabled",
    "_connect_sitl_command_mavlink",
    "_ensure_command_mavlink_connected",
    "command_link_connect_not_needed",
    "log_command_link_uses_servo_link",
]
