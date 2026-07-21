"""Pose and delta math for synthetic VISION_POSITION_DELTA messages."""

from __future__ import annotations

import numpy as np

from bridge.sitl_external_nav_vpd_delta_math import (
    compute_vpd_body_delta,
    initial_vpd_delta,
    vpd_delta_dt,
)
from bridge.sitl_external_nav_vpd_history import (
    store_vpd_current_state,
    take_vpd_previous_state,
    warn_stale_vpd_sample,
)
from bridge.sitl_math import quat_to_rotmat
from bridge.sitl_types import VerticalEstimate


def _external_nav_pose_rotation(
    vertical_est: VerticalEstimate,
    quat_ned_bfrd: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    extnav_pos = vertical_est.extnav_pos_ned if vertical_est.extnav_pos_ned is not None else vertical_est.pos_ned
    pos_ned = np.asarray(extnav_pos, dtype=np.float64)
    quat = np.asarray(quat_ned_bfrd, dtype=np.float64)
    if (
        pos_ned.shape[0] < 3
        or quat.shape[0] < 4
        or not np.all(np.isfinite(pos_ned[:3]))
        or not np.all(np.isfinite(quat[:4]))
    ):
        return None
    rot_ned_bfrd = quat_to_rotmat(quat[:4])
    if rot_ned_bfrd is None:
        return None
    return pos_ned, rot_ned_bfrd


def _external_nav_delta(
    self,
    vpd_clock_t: float,
    interval_s: float,
    pos_ned: np.ndarray,
    rot_ned_bfrd: np.ndarray,
    now_wall: float,
) -> tuple[float, np.ndarray, np.ndarray] | None:
    prev_t, prev_pos_ned, prev_rot_ned_bfrd = take_vpd_previous_state(self)
    store_vpd_current_state(self, vpd_clock_t=vpd_clock_t, pos_ned=pos_ned, rot_ned_bfrd=rot_ned_bfrd)
    if prev_t is None or prev_pos_ned is None or prev_rot_ned_bfrd is None:
        return initial_vpd_delta(interval_s)

    dt_s = vpd_delta_dt(vpd_clock_t, prev_t)
    if dt_s is None:
        return None
    if dt_s > 0.5:
        warn_stale_vpd_sample(self, dt_s=dt_s, now_wall=now_wall)
        return None

    return compute_vpd_body_delta(
        dt_s=dt_s,
        pos_ned=pos_ned,
        rot_ned_bfrd=rot_ned_bfrd,
        prev_pos_ned=prev_pos_ned,
        prev_rot_ned_bfrd=prev_rot_ned_bfrd,
    )


__all__ = ["_external_nav_delta", "_external_nav_pose_rotation"]
