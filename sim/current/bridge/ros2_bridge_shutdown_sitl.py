"""SITL transport shutdown helper for Ros2Bridge."""

from __future__ import annotations


def shutdown_sitl_transport(self) -> None:
    if self._sitl_transport is not None:
        with self._sitl_transport_lock:
            self._sitl_transport.shutdown()
        self._sitl_transport = None


__all__ = ["shutdown_sitl_transport"]
