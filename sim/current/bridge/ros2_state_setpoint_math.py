"""MAVROS setpoint math helpers for Ros2Bridge."""

from __future__ import annotations

import time

import numpy as np

from .ros2_math import quat_to_yaw, rotmat_to_quat_wxyz, wrap_angle_rad


def mavros_setpoint_active(owner) -> bool:
    return bool(owner._mavros_setpoint_enabled and owner._sitl_transport is None and owner._mavros_setpoint_last_t >= 0.0)


def mavros_setpoint_expired(owner, *, now: float | None = None) -> bool:
    stamp = time.monotonic() if now is None else float(now)
    return bool((stamp - owner._mavros_setpoint_last_t) > owner._mavros_setpoint_timeout_s)


def clear_mavros_setpoint(owner) -> None:
    owner._mavros_setpoint_pos = None
    owner._mavros_setpoint_yaw = None
    owner._mavros_pending_yaw_delta = 0.0
    owner._clear_cmd()


def current_yaw_ned(owner, base_rot_enu: np.ndarray) -> float:
    return quat_to_yaw(rotmat_to_quat_wxyz(owner._enu_to_ned @ base_rot_enu))


def apply_pending_yaw_delta(owner, base_rot_enu: np.ndarray) -> None:
    if owner._mavros_pending_yaw_delta == 0.0:
        return
    if owner._mavros_setpoint_yaw is None:
        owner._mavros_setpoint_yaw = float(current_yaw_ned(owner, base_rot_enu))
    owner._mavros_setpoint_yaw = float(owner._mavros_setpoint_yaw + owner._mavros_pending_yaw_delta)
    owner._mavros_pending_yaw_delta = 0.0


def mavros_position_commands(owner, base_pos_enu: np.ndarray, base_rot_enu: np.ndarray) -> tuple[float, float, float]:
    if owner._mavros_setpoint_pos is None:
        return 0.0, 0.0, 0.0
    target_enu = owner._enu_to_ned @ owner._mavros_setpoint_pos
    err_world = target_enu - base_pos_enu
    err_body = base_rot_enu.T @ err_world
    fwd_cmd = owner._mavros_setpoint_pos_kp * err_body[0]
    sway_cmd = owner._mavros_setpoint_pos_kp * err_body[1]
    heave_cmd = -owner._mavros_setpoint_heave_kp * err_body[2]
    return float(fwd_cmd), float(sway_cmd), float(heave_cmd)


def mavros_yaw_command(owner, base_rot_enu: np.ndarray) -> float:
    if owner._mavros_setpoint_yaw is None:
        return 0.0
    yaw_err = wrap_angle_rad(owner._mavros_setpoint_yaw - current_yaw_ned(owner, base_rot_enu))
    return float(owner._mavros_setpoint_yaw_kp * yaw_err)


__all__ = [
    "apply_pending_yaw_delta",
    "clear_mavros_setpoint",
    "current_yaw_ned",
    "mavros_position_commands",
    "mavros_setpoint_active",
    "mavros_setpoint_expired",
    "mavros_yaw_command",
]
