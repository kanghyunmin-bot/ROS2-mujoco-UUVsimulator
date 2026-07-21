"""Hydrostatic roll/pitch restoring torque helper."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from sim.runtime.pose_math import rpy_rad_from_quat_wxyz


def restoring_tau_world(runtime: Any, base_rot: np.ndarray) -> np.ndarray:
    hs = runtime.hydrostatic
    data = runtime.data
    quat = data.qpos[runtime.world_qpos_adr + 3 : runtime.world_qpos_adr + 7].copy()
    roll, pitch, _ = rpy_rad_from_quat_wxyz(quat)
    roll_trim_rad = float(hs.hydrostatic_restoring_roll_trim_rad)
    pitch_trim_rad = float(hs.hydrostatic_restoring_pitch_trim_rad)
    release_sim_time = runtime.initial_depth_hold.get("release_sim_time")
    if (
        hs.hydrostatic_restoring_release_trim_blend_s > 1.0e-9
        and release_sim_time is not None
        and math.isfinite(float(release_sim_time))
        and math.isfinite(hs.real_start_restoring_roll_trim_rad)
        and math.isfinite(hs.real_start_restoring_pitch_trim_rad)
    ):
        elapsed_since_release = max(0.0, float(data.time) - float(release_sim_time))
        if elapsed_since_release < hs.hydrostatic_restoring_release_trim_blend_s:
            start_weight = 1.0 - (elapsed_since_release / hs.hydrostatic_restoring_release_trim_blend_s)
            roll_trim_rad = (
                start_weight * float(hs.real_start_restoring_roll_trim_rad)
                + (1.0 - start_weight) * float(hs.hydrostatic_restoring_profile_roll_trim_rad)
            )
            pitch_trim_rad = (
                start_weight * float(hs.real_start_restoring_pitch_trim_rad)
                + (1.0 - start_weight) * float(hs.hydrostatic_restoring_profile_pitch_trim_rad)
            )
    roll_error = math.atan2(math.sin(float(roll) - roll_trim_rad), math.cos(float(roll) - roll_trim_rad))
    pitch_error = math.atan2(math.sin(float(pitch) - pitch_trim_rad), math.cos(float(pitch) - pitch_trim_rad))
    restoring_tau_body = np.array(
        [
            -hs.hydrostatic_restoring_roll_stiffness * roll_error,
            -hs.hydrostatic_restoring_pitch_stiffness * pitch_error,
            0.0,
        ],
        dtype=np.float64,
    )
    return base_rot @ restoring_tau_body


__all__ = ["restoring_tau_world"]
