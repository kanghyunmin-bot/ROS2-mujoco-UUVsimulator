"""Bar30 and vertical-state builders for SITL sensor snapshots."""

from __future__ import annotations

import mujoco
import numpy as np

from .ros2_sitl_sensor_vertical_hold import (
    bar30_velocity_after_initial_hold,
    vertical_estimate_after_initial_hold,
)
from .ros2_sitl_sensor_types import Bar30VerticalState, BaseKinematicState


def _bar30_position_enu(self, data: mujoco.MjData, base: BaseKinematicState) -> np.ndarray:
    depth_pos = self._sensor_slice(self.model, self.sensor_ids, "depth_pos", data)
    if depth_pos is not None and depth_pos.size >= 3 and np.all(np.isfinite(depth_pos[:3])):
        return np.array(depth_pos[:3], dtype=np.float64)
    return self._site_world_pos_enu(data, self._bar30_site_id, base.base_pos_enu)


def build_bar30_vertical_state(self, data: mujoco.MjData, base: BaseKinematicState) -> Bar30VerticalState:
    bar30_pos_enu = _bar30_position_enu(self, data, base)
    bar30_vel_enu = self._object_world_linear_velocity_enu(
        data,
        mujoco.mjtObj.mjOBJ_SITE,
        self._bar30_site_id,
        base.base_vel_enu,
    )
    bar30_vel_enu = bar30_velocity_after_initial_hold(
        self,
        bar30_vel_enu=bar30_vel_enu,
        bar30_pos_enu=bar30_pos_enu,
        base=base,
    )

    vertical_est = self._estimate_sitl_vertical(
        base.base_pos_enu,
        base.base_vel_enu,
        bar30_pos_enu,
        bar30_vel_enu,
        base.sim_t,
    )
    vertical_est = vertical_estimate_after_initial_hold(base=base, vertical_estimate=vertical_est)

    bar30_pressure_pa = float(
        self._estimate_bar30_pressure_pa(bar30_pos_enu, self._ros_bar30_depth_sensor_bias_m)
    )
    ros_depth_m = self._baro_pressure_law.frontend_depth_m_from_pressure(bar30_pressure_pa)
    return Bar30VerticalState(
        bar30_pos_enu=bar30_pos_enu,
        bar30_vel_enu=bar30_vel_enu,
        vertical_estimate=vertical_est,
        bar30_pressure_pa=bar30_pressure_pa,
        ros_depth_m=float(ros_depth_m),
    )


__all__ = ["build_bar30_vertical_state"]
