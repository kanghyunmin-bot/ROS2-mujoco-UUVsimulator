"""VISION_POSITION_DELTA clock and history reset helpers."""

from __future__ import annotations


def _reset_vpd_history_after_replay_rewind(self, sim_t: float, now_wall: float) -> None:
    self._sitl_extnav_last_send_sim_t = -1.0
    self._sitl_vpd_prev_clock_t = None
    self._sitl_vpd_prev_pos_ned = None
    self._sitl_vpd_prev_rot_ned_bfrd = None
    self._sitl_extnav_last_send_wall = now_wall
    if self._sitl_cmd_debug and now_wall - self._sitl_extnav_last_log_wall >= 2.0:
        print(
            "[sitl_transport] ExternalNav VPD history reset after "
            f"replay timestamp rewind to {float(sim_t):.3f}s",
            flush=True,
        )
        self._sitl_extnav_last_log_wall = now_wall


def _vpd_clock(self, sim_t: float, now_wall: float) -> tuple[float, float]:
    vpd_clock_t = (
        float(max(0.0, now_wall - self._sitl_extnav_start_wall))
        if self._sitl_extnav_scheduler == "wall_time"
        else float(max(0.0, sim_t))
    )
    last_vpd_clock_t = (
        float(max(0.0, self._sitl_extnav_last_send_wall - self._sitl_extnav_start_wall))
        if self._sitl_extnav_scheduler == "wall_time" and self._sitl_extnav_last_send_wall > 0.0
        else float(self._sitl_extnav_last_send_sim_t)
    )
    return vpd_clock_t, last_vpd_clock_t


__all__ = ["_reset_vpd_history_after_replay_rewind", "_vpd_clock"]
