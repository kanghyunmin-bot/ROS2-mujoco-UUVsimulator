"""Send-status diagnostic logging for ArduSub JSON sensor sends."""

from __future__ import annotations


def log_sitl_send_status_due(self, *, now_wall: float, target, sent: int, prev_target) -> None:
    if now_wall - self._sitl_last_send_wall > 5.0:
        _log_periodic_send_status(self, target=target, sent=sent, now_wall=now_wall)
        self._sitl_last_send_wall = now_wall

    if self._sitl_cmd_debug and prev_target is not None and prev_target != target:
        print(f"[sitl_transport] SITL send target changed to servo source: {target}", flush=True)

    if now_wall - self._sitl_last_send_err_wall > 20.0 and sent <= 0:
        print(f"[sitl_transport] SITL send warning: sent {sent} bytes to {target}", flush=True)
        self._sitl_last_send_err_wall = now_wall


def log_sitl_send_failure_due(self, *, now_wall: float, target, exc: Exception) -> None:
    if now_wall - self._sitl_last_send_err_wall > 2.0:
        print(f"[sitl_transport] SITL send failed to {target}: {exc}", flush=True)
        self._sitl_last_send_err_wall = now_wall
    self._sitl_last_send_wall = now_wall


def _log_periodic_send_status(self, *, target, sent: int, now_wall: float) -> None:
    if self._sitl_client_addr is None:
        print(
            f"[sitl_transport] SITL send target still default (endpoint not discovered yet): {target} "
            f"packets_sent={self._sitl_send_counter}",
            flush=True,
        )
        self._sitl_last_no_client_wall = now_wall
    elif self._sitl_cmd_debug:
        print(
            f"[sitl_transport] SITL send ok (sensor_target={target}, bytes={sent}, "
            f"packets_sent={self._sitl_send_counter})",
            flush=True,
        )


__all__ = ["log_sitl_send_failure_due", "log_sitl_send_status_due"]
