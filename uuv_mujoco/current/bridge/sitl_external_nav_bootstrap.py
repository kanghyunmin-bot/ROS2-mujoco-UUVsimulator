"""ExternalNav bootstrap messages for SITL MAVLink."""

from __future__ import annotations

import time


def _send_external_nav_bootstrap(self, sim_t: float, now_wall: float, mav) -> None:
    if mav is None:
        return
    if self._sitl_extnav_bootstrap_count >= 5:
        return
    if now_wall - self._sitl_extnav_last_bootstrap_wall < 1.0:
        return
    target = self._resolve_mav_target()
    if target is None:
        return
    target_sys, _target_comp = target
    try:
        mav.mav.system_time_send(
            int(time.time() * 1.0e6),
            int(max(0.0, sim_t) * 1000.0) & 0xFFFFFFFF,
        )
        mav.mav.set_gps_global_origin_send(
            int(target_sys),
            int(self._sitl_extnav_origin_lat_e7),
            int(self._sitl_extnav_origin_lon_e7),
            int(self._sitl_extnav_origin_alt_mm),
        )
        self._sitl_extnav_bootstrap_count += 1
        self._sitl_extnav_last_bootstrap_wall = now_wall
    except Exception as exc:
        if now_wall - self._sitl_extnav_send_failed_wall > 2.0:
            print(f"[sitl_transport] ExternalNav bootstrap send failed: {exc}", flush=True)
            self._sitl_extnav_send_failed_wall = now_wall


__all__ = ["_send_external_nav_bootstrap"]
