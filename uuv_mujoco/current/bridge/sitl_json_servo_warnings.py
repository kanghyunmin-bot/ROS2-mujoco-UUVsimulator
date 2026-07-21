"""Client-missing and stale JSON servo warnings."""

from __future__ import annotations


def warn_json_servo_client_status(self, now_wall: float) -> None:
    if self._sitl_client_addr is None:
        if now_wall - self._sitl_last_client_missing_wall >= self._sitl_no_client_warn_interval_s:
            print(
                f"[sitl_transport] Waiting for SITL servo packets on {self.sitl_addr} "
                "(QGC joystick enabled, vehicle armed, and mode MANUAL/STABILIZE/DEPTH_HOLD).",
                flush=True,
            )
            self._sitl_last_client_missing_wall = now_wall
        return
    if self._sitl_client_last_wall <= 0.0:
        return
    if now_wall - self._sitl_client_last_wall < self._sitl_no_client_warn_interval_s:
        return
    if now_wall - self._sitl_last_command_stale_wall < self._sitl_no_client_warn_interval_s:
        return
    print(
        f"[sitl_transport] No new SITL servo packets for "
        f"{now_wall - self._sitl_client_last_wall:.1f}s from {self._sitl_client_addr}",
        flush=True,
    )
    self._sitl_last_command_stale_wall = now_wall


__all__ = ["warn_json_servo_client_status"]
