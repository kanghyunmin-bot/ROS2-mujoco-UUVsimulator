"""GCS heartbeat send helper for SITL MAVLink command paths."""

from __future__ import annotations


def _send_gcs_heartbeat(self, *, force: bool = False, mav=None) -> None:
    mav = mav if mav is not None else self._mav_for_commands()
    if mav is None or self._sitl_mavutil is None:
        return
    link = self._command_link_for_mav(mav)
    if link is None:
        return
    link.send_gcs_heartbeat(self._sitl_mavutil, force=force)
    sync_heartbeat_timestamps(self)


def sync_heartbeat_timestamps(self) -> None:
    self._sitl_cmd_mav_last_heartbeat_send_wall = self._sitl_command_link.last_heartbeat_send_wall
    self._sitl_mav_last_heartbeat_send_wall = self._sitl_servo_command_link.last_heartbeat_send_wall


__all__ = ["_send_gcs_heartbeat", "sync_heartbeat_timestamps"]
