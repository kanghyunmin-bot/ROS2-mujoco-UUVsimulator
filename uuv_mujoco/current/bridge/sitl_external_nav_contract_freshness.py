"""ExternalNav native VPD replay freshness checks."""

from __future__ import annotations


def _native_vpd_fresh_enough(self) -> bool:
    if not self._native_vpd_events:
        return False
    if not self._sensor_replay_ready or self._sensor_replay_current_t_s is None:
        return True
    current_replay_t = float(self._sensor_replay_current_t_s)
    first_event_t = float(self._native_vpd_events[0].t_replay_s)
    if current_replay_t < first_event_t:
        return True
    if self._native_vpd_last_replay_t_s is None:
        return False
    replay_stale_s = current_replay_t - float(self._native_vpd_last_replay_t_s)
    return replay_stale_s <= self._sitl_extnav_max_stale_s


__all__ = ["_native_vpd_fresh_enough"]
