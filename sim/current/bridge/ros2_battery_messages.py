"""Battery-state ROS2 message builders."""

from __future__ import annotations

from typing import Any

import numpy as np


def build_battery_msg(
    battery_state_type: type,
    stamp: Any,
    *,
    voltage: float,
    current: float,
    state_of_charge_percent: float,
) -> Any:
    msg = battery_state_type()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.voltage = float(voltage)
    msg.current = float(current)
    msg.percentage = float(np.clip(state_of_charge_percent, 0.0, 100.0) / 100.0)
    msg.charge = -1.0
    msg.capacity = -1.0
    return msg


__all__ = ["build_battery_msg"]
