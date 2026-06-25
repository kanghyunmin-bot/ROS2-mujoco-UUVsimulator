"""Synthetic VISION_POSITION_DELTA sender from MuJoCo truth pose."""

from __future__ import annotations

import time

import numpy as np

from bridge.sitl_external_nav_vpd_emit import _emit_external_nav_vpd
from bridge.sitl_external_nav_vpd_prepare import prepare_external_nav_vpd_sample
from bridge.sitl_types import VerticalEstimate


def _send_external_nav(
    self,
    sim_t: float,
    vertical_est: VerticalEstimate,
    quat_ned_bfrd: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    if not self._sitl_extnav_enabled:
        return
    self._ensure_command_mavlink_connected()
    mav = self._mav_for_external_nav()
    if mav is None:
        return
    now_wall = time.monotonic()
    self._send_external_nav_bootstrap(sim_t, now_wall, mav)
    if self._send_native_vision_delta_due(mav, now_wall):
        return
    sample = prepare_external_nav_vpd_sample(
        self,
        sim_t=sim_t,
        now_wall=now_wall,
        vertical_est=vertical_est,
        quat_ned_bfrd=quat_ned_bfrd,
    )
    if sample is None:
        return

    _emit_external_nav_vpd(
        self,
        mav=mav,
        sim_t=sim_t,
        now_wall=now_wall,
        vpd_clock_t=sample.vpd_clock_t,
        dt_s=sample.dt_s,
        pos_delta_body=sample.pos_delta_body,
        angle_delta_body=sample.angle_delta_body,
        pos_ned=sample.pos_ned,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


__all__ = ["_send_external_nav"]
