"""Base-body kinematics for SITL sensor snapshots."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_sitl_sensor_types import BaseKinematicState


def build_base_kinematic_state(self, data: mujoco.MjData) -> BaseKinematicState | None:
    sim_t = float(data.time)
    if self._base_id < 0:
        return None

    try:
        base_pos_enu = np.array(data.xpos[self._base_id], dtype=np.float64)
        base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
        quat_base = np.array(data.xquat[self._base_id], dtype=np.float64)
    except Exception:
        return None

    # Keep ROS/SITL kinematics on the same velocity contract used by the
    # physics debug path: MuJoCo body-local linear velocity rotated to world.
    # mj_objectVelocity(..., flg_local=0) is easy to misinterpret here and was
    # causing published body velocity axes to diverge from the plant axes.
    base_vel_enu = self._body_cvel_world_linear_velocity_enu(data, self._base_id, base_rot_enu)
    zero_vertical_reason = self._sitl_vertical_feedback_zero_reason()
    if self._sitl_initial_depth_hold_active:
        base_vel_enu = np.zeros(3, dtype=np.float64)
        self._sitl_prev_vel_sim_t = sim_t
        self._sitl_prev_vel_enu = base_vel_enu.copy()
    elif zero_vertical_reason:
        base_vel_enu = np.asarray(base_vel_enu, dtype=np.float64).copy()
        base_vel_enu[2] = 0.0
        self._sitl_prev_vel_sim_t = sim_t
        self._sitl_prev_vel_enu = base_vel_enu.copy()
        self._log_sitl_vertical_feedback_zero(zero_vertical_reason)

    return BaseKinematicState(
        sim_t=sim_t,
        base_pos_enu=base_pos_enu,
        base_rot_enu=base_rot_enu,
        quat_base=quat_base,
        base_vel_enu=base_vel_enu,
        zero_vertical_reason=zero_vertical_reason,
    )


__all__ = ["build_base_kinematic_state"]
