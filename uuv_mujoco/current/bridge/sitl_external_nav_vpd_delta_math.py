"""Delta math for synthetic ExternalNav VPD messages."""

from __future__ import annotations

import numpy as np

from bridge.sitl_math import rotmat_to_rpy


def initial_vpd_delta(interval_s: float) -> tuple[float, np.ndarray, np.ndarray]:
    return interval_s, np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)


def vpd_delta_dt(vpd_clock_t: float, prev_t: float | None) -> float | None:
    if prev_t is None:
        return None
    dt_s = float(vpd_clock_t) - float(prev_t)
    if not np.isfinite(dt_s) or dt_s < 1.0e-4:
        return None
    return dt_s


def compute_vpd_body_delta(
    *,
    dt_s: float,
    pos_ned: np.ndarray,
    rot_ned_bfrd: np.ndarray,
    prev_pos_ned: np.ndarray,
    prev_rot_ned_bfrd: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray] | None:
    # ArduSub's AP_VisualOdom VPD path expects deltas in current BODY_FRD:
    # SIM_Vicon computes body_from_ned_current * (pos_current - pos_prev).
    pos_delta_ned = pos_ned[:3] - prev_pos_ned[:3]
    pos_delta_body = rot_ned_bfrd.T @ pos_delta_ned
    rot_delta_prev_to_curr_body = rot_ned_bfrd.T @ prev_rot_ned_bfrd
    angle_delta_body = np.asarray(rotmat_to_rpy(rot_delta_prev_to_curr_body), dtype=np.float64)
    if not np.all(np.isfinite(pos_delta_body)) or not np.all(np.isfinite(angle_delta_body)):
        return None
    return dt_s, pos_delta_body, angle_delta_body


__all__ = [
    "compute_vpd_body_delta",
    "initial_vpd_delta",
    "vpd_delta_dt",
]
