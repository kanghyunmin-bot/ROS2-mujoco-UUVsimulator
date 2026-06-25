"""Transmit-due gate for synthetic VISION_POSITION_DELTA."""

from __future__ import annotations

from bridge.sitl_external_nav_vpd_clock import _reset_vpd_history_after_replay_rewind, _vpd_clock


def _external_nav_vpd_due(self, sim_t: float, now_wall: float) -> tuple[float, float] | None:
    if (
        self._sitl_extnav_last_send_sim_t >= 0.0
        and float(sim_t) + 1.0e-3 < float(self._sitl_extnav_last_send_sim_t)
    ):
        _reset_vpd_history_after_replay_rewind(self, sim_t, now_wall)

    vpd_clock_t, last_vpd_clock_t = _vpd_clock(self, sim_t, now_wall)
    interval_s = 1.0 / max(self._sitl_extnav_rate_hz, 1.0)
    if last_vpd_clock_t >= 0.0 and vpd_clock_t - last_vpd_clock_t < interval_s:
        return None
    return vpd_clock_t, interval_s


__all__ = ["_external_nav_vpd_due"]
