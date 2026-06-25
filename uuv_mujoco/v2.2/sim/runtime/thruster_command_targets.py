"""Direct command to thruster target conversion."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np


def apply_direct_command_targets(
    runtime: Any,
    *,
    command_state: Any,
    mix_horizontal_thrusters: Callable[[float, float, float], np.ndarray],
    vertical_names: list[str],
    horizontal_order: list[str],
) -> tuple[float, float, float, float]:
    """Convert recent ROS bridge commands into thruster targets."""

    fwd_cmd, sway_cmd, yaw_cmd, heave_cmd = command_state.normalized()
    horiz_cmd = mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)

    for name in runtime.all_thruster_names:
        runtime.target[name] = 0.0
    for name in vertical_names:
        runtime.target[name] = heave_cmd
    for i, name in enumerate(horizontal_order):
        runtime.target[name] = float(horiz_cmd[i])
    return fwd_cmd, sway_cmd, yaw_cmd, heave_cmd


__all__ = ["apply_direct_command_targets"]
