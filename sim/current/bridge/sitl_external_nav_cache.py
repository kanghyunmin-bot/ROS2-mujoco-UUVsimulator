"""Wall-clock ExternalNav cache helpers."""

from __future__ import annotations

import numpy as np

from bridge.sitl_types import VerticalEstimate


def _cache_external_nav_state(
    self,
    sim_t: float,
    vertical_est: VerticalEstimate,
    quat_ned_bfrd: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    if not _uses_live_wall_external_nav(self):
        return
    self._sitl_extnav_latest_state = (
        float(max(0.0, sim_t)),
        _copy_vertical_estimate(vertical_est),
        np.asarray(quat_ned_bfrd, dtype=np.float64).copy(),
        float(roll),
        float(pitch),
        float(yaw),
    )


def send_cached_external_nav_due(self) -> None:
    """Feed live ExternalNav at wall-clock rate independent of viewer FPS."""

    if not _uses_live_wall_external_nav(self):
        return
    state = self._sitl_extnav_latest_state
    if state is None:
        return
    sim_t, vertical_est, quat, roll, pitch, yaw = state
    self._send_external_nav(sim_t, vertical_est, quat, roll, pitch, yaw)


@property
def live_wall_external_nav(self) -> bool:
    return _uses_live_wall_external_nav(self)


def _uses_live_wall_external_nav(self) -> bool:
    return bool(
        self._sitl_extnav_enabled
        and self._sitl_extnav_scheduler == "wall_time"
        and not self._sensor_replay_frames
        and not self._native_vpd_events
    )


def _copy_vertical_estimate(vertical_est: VerticalEstimate) -> VerticalEstimate:
    return VerticalEstimate(
        depth_m=float(vertical_est.depth_m),
        pressure_pa=None if vertical_est.pressure_pa is None else float(vertical_est.pressure_pa),
        pos_ned=np.asarray(vertical_est.pos_ned, dtype=np.float64).copy(),
        vel_ned=np.asarray(vertical_est.vel_ned, dtype=np.float64).copy(),
        alt_m=float(vertical_est.alt_m),
        extnav_pos_ned=(
            None
            if vertical_est.extnav_pos_ned is None
            else np.asarray(vertical_est.extnav_pos_ned, dtype=np.float64).copy()
        ),
    )


__all__ = ["_cache_external_nav_state", "live_wall_external_nav", "send_cached_external_nav_due"]
