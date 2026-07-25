"""MAVROS manual-control callback for Ros2Bridge."""

from __future__ import annotations

import numpy as np


def _manual_control_axis_to_norm(value: object) -> float:
    axis = float(value)
    if abs(axis) > 1.0:
        axis /= 1000.0
    return float(np.clip(axis, -1.0, 1.0))


def _manual_control_heave_to_norm(value: object) -> float:
    heave = float(value)
    if abs(heave) > 1.0:
        heave = (heave - 500.0) / 500.0
    return float(np.clip(heave, -1.0, 1.0))


def _on_mavros_manual_control(self, msg) -> None:
    x = _manual_control_axis_to_norm(getattr(msg, "x", 0.0))
    y = _manual_control_axis_to_norm(getattr(msg, "y", 0.0))
    z = _manual_control_heave_to_norm(getattr(msg, "z", 0.0))
    r = _manual_control_axis_to_norm(getattr(msg, "r", 0.0))
    buttons = int(getattr(msg, "buttons", 0))
    if self._sitl_transport is not None:
        with self._sitl_transport_lock:
            self._sitl_transport.send_manual_control(x=x, y=y, z=z, r=r, buttons=buttons)
        return
    self._handle_normalized_cmd(x, y, r, z)


__all__ = [
    "_manual_control_axis_to_norm",
    "_manual_control_heave_to_norm",
    "_on_mavros_manual_control",
]
