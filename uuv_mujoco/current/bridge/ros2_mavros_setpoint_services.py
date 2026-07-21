"""MAVROS setpoint and COMMAND_LONG callbacks for Ros2Bridge."""

from __future__ import annotations

import time

from .ros2_mavros_condition_yaw import apply_condition_yaw_command
from .ros2_mavros_setpoint_position import position_target_ned


def _on_mavros_setpoint(self, msg) -> None:
    if not self._mavros_setpoint_enabled:
        return
    frame_local_ned = getattr(self.PositionTarget, "FRAME_LOCAL_NED", 1)
    coordinate_frame = int(getattr(msg, "coordinate_frame", frame_local_ned))
    if coordinate_frame != frame_local_ned:
        return
    tmask = int(getattr(msg, "type_mask", 0))
    target_ned, has_pos = position_target_ned(self, msg, tmask)
    if has_pos:
        self._mavros_setpoint_pos = target_ned.copy()
    if not (tmask & self._POSITION_TARGET_TYPEMASK_YAW_IGNORE):
        self._mavros_setpoint_yaw = float(getattr(msg, "yaw", 0.0))
    self._mavros_setpoint_last_t = time.monotonic()


def _on_mavros_command_long(self, request, response):
    command = int(getattr(request, "command", 0))
    if command == self._MAV_CMD_CONDITION_YAW:
        apply_condition_yaw_command(self, request)
    if hasattr(response, "success"):
        response.success = True
    if hasattr(response, "result"):
        response.result = 0
    return response


__all__ = ["_on_mavros_command_long", "_on_mavros_setpoint"]
