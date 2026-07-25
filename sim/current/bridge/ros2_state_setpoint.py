"""MAVROS setpoint command helper for Ros2Bridge."""

from __future__ import annotations

import numpy as np

from .ros2_state_setpoint_math import (
    apply_pending_yaw_delta,
    clear_mavros_setpoint,
    mavros_position_commands,
    mavros_setpoint_active,
    mavros_setpoint_expired,
    mavros_yaw_command,
)


def _apply_mavros_setpoint(self, base_pos_enu: np.ndarray, base_rot_enu: np.ndarray) -> None:
    if not mavros_setpoint_active(self):
        return
    if mavros_setpoint_expired(self):
        clear_mavros_setpoint(self)
        return

    apply_pending_yaw_delta(self, base_rot_enu)
    fwd_cmd, sway_cmd, heave_cmd = mavros_position_commands(self, base_pos_enu, base_rot_enu)
    yaw_cmd = mavros_yaw_command(self, base_rot_enu)
    self._handle_normalized_cmd(float(fwd_cmd), float(sway_cmd), float(yaw_cmd), float(heave_cmd))


__all__ = ["_apply_mavros_setpoint"]
