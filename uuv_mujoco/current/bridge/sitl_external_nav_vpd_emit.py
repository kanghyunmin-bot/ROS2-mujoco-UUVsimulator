"""MAVLink emit step for synthetic VISION_POSITION_DELTA."""

from __future__ import annotations

import numpy as np

from bridge.sitl_external_nav_vpd_rate import _check_external_nav_tx_rate


def _log_external_nav_sample(
    self,
    *,
    now_wall: float,
    dt_s: float,
    pos_delta_body: np.ndarray,
    angle_delta_body: np.ndarray,
    pos_ned: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    if not (self._sitl_cmd_debug and now_wall - self._sitl_extnav_last_log_wall >= 2.0):
        return
    print(
        "[sitl_transport] ExternalNav VPD tx sample "
        f"dt={dt_s:.3f}s pos_delta_body={pos_delta_body[:3].tolist()} "
        f"angle_delta_body={angle_delta_body[:3].tolist()} "
        f"truth_pos_ned={pos_ned[:3].tolist()} "
        f"rpy={[float(roll), float(pitch), float(yaw)]}",
        flush=True,
    )
    self._sitl_extnav_last_log_wall = now_wall


def _emit_external_nav_vpd(
    self,
    *,
    mav,
    sim_t: float,
    now_wall: float,
    vpd_clock_t: float,
    dt_s: float,
    pos_delta_body: np.ndarray,
    angle_delta_body: np.ndarray,
    pos_ned: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    try:
        usec = int(max(0.0, float(vpd_clock_t)) * 1.0e6)
        time_delta_usec = int(np.clip(round(dt_s * 1.0e6), 1, 500000))
        mav.mav.vision_position_delta_send(
            usec,
            time_delta_usec,
            [float(x) for x in angle_delta_body[:3]],
            [float(x) for x in pos_delta_body[:3]],
            float(self._sitl_vpd_confidence),
        )
        self._sitl_extnav_last_send_sim_t = float(sim_t)
        self._sitl_extnav_last_send_wall = now_wall
        self._sitl_extnav_tx_window_count += 1
        _check_external_nav_tx_rate(self, now_wall)
        _log_external_nav_sample(
            self,
            now_wall=now_wall,
            dt_s=dt_s,
            pos_delta_body=pos_delta_body,
            angle_delta_body=angle_delta_body,
            pos_ned=pos_ned,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
    except Exception as exc:
        if now_wall - self._sitl_extnav_send_failed_wall > 2.0:
            print(f"[sitl_transport] ExternalNav send failed: {exc}", flush=True)
            self._sitl_extnav_send_failed_wall = now_wall


__all__ = ["_emit_external_nav_vpd", "_log_external_nav_sample"]
