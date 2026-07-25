"""Socket connection and shutdown helpers for SitlTransport."""

from __future__ import annotations

from .qgc_mavlink_relay import close_qgc_mavlink_relay


def _connect_sitl(self) -> None:
    """Initialize UDP socket for ArduPilot SITL JSON interface."""
    try:
        self.sitl_sock = self._json_servo_receiver.connect()
        sock_addr = self.sitl_sock.getsockname()
        print(
            f"[sitl_transport] SITL socket initialized (listen {sock_addr}, "
            f"servo_target={self.sitl_addr}, sensor_target={self.sitl_send_addr})",
            flush=True,
        )
    except Exception as exc:
        print(f"[sitl_transport] Failed to init SITL socket: {exc}", flush=True)


def _close_mav_connection(mav) -> None:
    try:
        close_fn = getattr(mav, "close", None)
        if callable(close_fn):
            close_fn()
    except Exception:
        pass


def shutdown(self) -> None:
    close_qgc_mavlink_relay(self)
    self._json_servo_receiver.close()
    self.sitl_sock = None
    if self._sitl_mav is not None:
        _close_mav_connection(self._sitl_mav)
        self._sitl_mav = None
        self._sitl_servo_command_link.set_connection(None)
    if self._sitl_cmd_mav is not None:
        _close_mav_connection(self._sitl_cmd_mav)
        self._sitl_cmd_mav = None
        self._sitl_command_link.set_connection(None)


__all__ = ["_connect_sitl", "shutdown"]
