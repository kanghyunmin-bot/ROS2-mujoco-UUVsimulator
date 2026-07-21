"""DVL body velocity assembly for SITL sensor snapshots."""

from __future__ import annotations

import os
import time
from typing import Any

import mujoco
import numpy as np

from .ros2_sitl_sensor_types import BaseKinematicState


def _base_velocity_body_bmj(owner: Any, data: Any, base: BaseKinematicState) -> np.ndarray:
    if getattr(owner, "_base_id", -1) >= 0:
        try:
            vel6 = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(
                owner.model,
                data,
                mujoco.mjtObj.mjOBJ_BODY,
                int(owner._base_id),
                vel6,
                1,
            )
            if np.all(np.isfinite(vel6[3:6])):
                return vel6[3:6].copy()
        except Exception:
            pass
    return base.base_rot_enu.T @ base.base_vel_enu


def _filtered_velocity(owner: Any, vel_body_bmj: np.ndarray) -> np.ndarray:
    vel_body_bmj = np.nan_to_num(np.asarray(vel_body_bmj, dtype=np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    alpha = float(owner._dvl_filter_alpha)
    if alpha <= 0.0:
        owner._dvl_vel_body_filt = vel_body_bmj
        return vel_body_bmj
    if owner._dvl_vel_body_filt is None:
        owner._dvl_vel_body_filt = vel_body_bmj
    else:
        owner._dvl_vel_body_filt = ((1.0 - alpha) * owner._dvl_vel_body_filt) + (alpha * vel_body_bmj)
    return owner._dvl_vel_body_filt.copy()


def dvl_velocity_from_snapshot(owner: Any, data: Any, base: BaseKinematicState, dvl_vel_sensor: Any, gyro_bmj: Any) -> np.ndarray | None:
    # Body velocity topics must describe base_link motion. MuJoCo site
    # velocimeters are expressed at the sensor site and include lever-arm and
    # site-frame effects, which makes manual axis checks look cross-coupled.
    dvl_vel_body_bmj = _filtered_velocity(owner, _base_velocity_body_bmj(owner, data, base))
    if os.environ.get("ROS2_UUV_DVL_DEBUG") == "1":
        now = time.monotonic()
        last = float(getattr(owner, "_dvl_contract_debug_last_wall", -10.0))
        if now - last >= 1.0:
            owner._dvl_contract_debug_last_wall = now
            sensor = None if dvl_vel_sensor is None else np.asarray(dvl_vel_sensor, dtype=np.float64)
            print(
                "[dvl-contract] "
                f"body_bmj={np.array2string(dvl_vel_body_bmj, precision=4)} "
                f"sensor={None if sensor is None else np.array2string(sensor, precision=4)}",
                flush=True,
            )
    if owner._sitl_initial_depth_hold_active:
        return np.zeros(3, dtype=np.float64)
    if base.zero_vertical_reason and dvl_vel_body_bmj is not None:
        dvl_vel_body_bmj = np.asarray(dvl_vel_body_bmj, dtype=np.float64).copy()
        dvl_vel_body_bmj[2] = 0.0
    return dvl_vel_body_bmj


__all__ = ["dvl_velocity_from_snapshot"]
