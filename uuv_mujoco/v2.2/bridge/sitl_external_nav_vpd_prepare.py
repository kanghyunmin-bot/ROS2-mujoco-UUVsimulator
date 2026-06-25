"""Prepare synthetic VISION_POSITION_DELTA samples before MAVLink emit."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from bridge.sitl_external_nav_vpd_due import _external_nav_vpd_due
from bridge.sitl_external_nav_vpd_pose import _external_nav_delta, _external_nav_pose_rotation
from bridge.sitl_types import VerticalEstimate


@dataclass(frozen=True)
class ExternalNavVpdSample:
    vpd_clock_t: float
    dt_s: float
    pos_delta_body: np.ndarray
    angle_delta_body: np.ndarray
    pos_ned: np.ndarray


def prepare_external_nav_vpd_sample(
    transport: object,
    *,
    sim_t: float,
    now_wall: float,
    vertical_est: VerticalEstimate,
    quat_ned_bfrd: np.ndarray,
) -> ExternalNavVpdSample | None:
    due = _external_nav_vpd_due(transport, sim_t, now_wall)
    if due is None:
        return None
    vpd_clock_t, interval_s = due

    pose_rotation = _external_nav_pose_rotation(vertical_est, quat_ned_bfrd)
    if pose_rotation is None:
        return None
    pos_ned, rot_ned_bfrd = pose_rotation

    delta = _external_nav_delta(transport, vpd_clock_t, interval_s, pos_ned, rot_ned_bfrd, now_wall)
    if delta is None:
        return None
    dt_s, pos_delta_body, angle_delta_body = delta
    return ExternalNavVpdSample(
        vpd_clock_t=vpd_clock_t,
        dt_s=dt_s,
        pos_delta_body=pos_delta_body,
        angle_delta_body=angle_delta_body,
        pos_ned=pos_ned,
    )


__all__ = ["ExternalNavVpdSample", "prepare_external_nav_vpd_sample"]
