"""ROS subscription callbacks for ALT_HOLD diagnostics."""

from __future__ import annotations

import time

from althold_diagnostics_depth_callbacks import (
    on_depth,
    on_dvl_vel,
    on_mavros_pose,
    on_mavros_vel,
    on_mujoco_pose,
    on_pressure,
    on_sim_odom,
)
from althold_diagnostics_rc_callbacks import (
    on_manual,
    on_rc_in,
    on_rc_out,
)
from althold_diagnostics_status_callbacks import on_sitl_mavlink_status, on_sitl_sensor_status


def _elapsed(self) -> float:
    return time.monotonic() - self.start_wall


def _on_manual(self, msg) -> None:
    on_manual(self, msg)


def _on_rc_in(self, msg) -> None:
    on_rc_in(self, msg)


def _on_rc_out(self, msg) -> None:
    on_rc_out(self, msg)


def _on_depth(self, msg) -> None:
    on_depth(self, msg)


def _on_pressure(self, msg) -> None:
    on_pressure(self, msg)


def _on_mavros_pose(self, msg) -> None:
    on_mavros_pose(self, msg)


def _on_mavros_vel(self, msg) -> None:
    on_mavros_vel(self, msg)


def _on_dvl_vel(self, msg) -> None:
    on_dvl_vel(self, msg)


def _on_mujoco_pose(self, msg) -> None:
    on_mujoco_pose(self, msg)


def _on_sim_odom(self, msg) -> None:
    on_sim_odom(self, msg)


def _on_sitl_mavlink_status(self, msg) -> None:
    on_sitl_mavlink_status(self, msg)


def _on_sitl_sensor_status(self, msg) -> None:
    on_sitl_sensor_status(self, msg)
