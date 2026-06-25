"""Locked SITL transport calls for MAVROS arm/mode requests."""

from __future__ import annotations


def send_arm_to_sitl(self, arm_value: bool) -> bool:
    if self._sitl_transport is None:
        return True
    with self._sitl_transport_lock:
        return bool(self._sitl_transport.send_arm_command(bool(arm_value)))


def send_mode_to_sitl(self, mode: str) -> bool:
    if not mode or self._sitl_transport is None:
        return True
    with self._sitl_transport_lock:
        return bool(self._sitl_transport.send_set_mode(mode))


__all__ = ["send_arm_to_sitl", "send_mode_to_sitl"]
