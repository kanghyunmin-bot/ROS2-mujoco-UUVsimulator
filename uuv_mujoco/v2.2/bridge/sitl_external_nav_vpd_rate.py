"""Transmission-rate contract checks for ExternalNav VPD."""

from __future__ import annotations


def _check_external_nav_tx_rate(self, now_wall: float) -> None:
    window_s = now_wall - self._sitl_extnav_tx_window_start_wall
    if window_s < 2.0:
        return
    rate_hz = self._sitl_extnav_tx_window_count / max(window_s, 1.0e-6)
    self._sitl_extnav_last_rate_hz = float(rate_hz)
    if (
        self._sitl_extnav_required
        and now_wall - self._sitl_extnav_start_wall > self._sitl_extnav_grace_s
        and rate_hz < self._sitl_extnav_min_tx_hz
    ):
        msg = (
            "ExternalNav TX rate below contract: "
            f"{rate_hz:.2f}Hz < {self._sitl_extnav_min_tx_hz:.2f}Hz"
        )
        if self._sitl_extnav_rate_fatal:
            self._sitl_extnav_fault = msg
        elif now_wall - self._sitl_extnav_rate_warn_wall > 3.0:
            print(f"[sitl_transport] warning: {msg}", flush=True)
            self._sitl_extnav_rate_warn_wall = now_wall
    self._sitl_extnav_tx_window_start_wall = now_wall
    self._sitl_extnav_tx_window_count = 0


__all__ = ["_check_external_nav_tx_rate"]
