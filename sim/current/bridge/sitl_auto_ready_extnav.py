"""ExternalNav readiness gate for SITL auto-ready."""

from __future__ import annotations


def _auto_ready_extnav_ready(self, now_wall: float) -> bool:
    if not self._sitl_extnav_required:
        return True
    if not self._sitl_extnav_enabled or self._sitl_extnav_fault:
        return False
    if self._sitl_extnav_last_send_wall <= 0.0:
        return False
    extnav_age_s = now_wall - self._sitl_extnav_last_send_wall
    if extnav_age_s > self._sitl_extnav_max_stale_s:
        return False
    extnav_grace_active = now_wall - self._sitl_extnav_start_wall < self._sitl_extnav_grace_s
    return bool(extnav_grace_active or self._sitl_extnav_last_rate_hz >= self._sitl_extnav_min_tx_hz)


__all__ = ["_auto_ready_extnav_ready"]
