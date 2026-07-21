"""Native VPD TX-rate contract monitoring."""

from __future__ import annotations


def update_native_vpd_rate_window(transport: object, *, now_wall: float) -> None:
    window_s = float(now_wall) - transport._sitl_extnav_tx_window_start_wall
    if window_s < 2.0:
        return

    rate_hz = transport._sitl_extnav_tx_window_count / max(window_s, 1.0e-6)
    transport._sitl_extnav_last_rate_hz = float(rate_hz)
    if (
        transport._sitl_extnav_required
        and now_wall - transport._sitl_extnav_start_wall > transport._sitl_extnav_grace_s
        and rate_hz < transport._sitl_extnav_min_tx_hz
        and transport._native_vpd_cursor > 0
    ):
        msg = (
            "ExternalNav native VPD TX rate below contract: "
            f"{rate_hz:.2f}Hz < {transport._sitl_extnav_min_tx_hz:.2f}Hz"
        )
        if transport._sitl_extnav_rate_fatal:
            transport._sitl_extnav_fault = msg
        elif now_wall - transport._sitl_extnav_rate_warn_wall > 3.0:
            print(f"[sitl_transport] warning: {msg}", flush=True)
            transport._sitl_extnav_rate_warn_wall = now_wall

    transport._sitl_extnav_tx_window_start_wall = now_wall
    transport._sitl_extnav_tx_window_count = 0


__all__ = ["update_native_vpd_rate_window"]
