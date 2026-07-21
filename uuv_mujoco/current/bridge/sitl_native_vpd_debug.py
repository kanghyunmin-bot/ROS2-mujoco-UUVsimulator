"""Native VPD debug logging."""

from __future__ import annotations


def log_native_vpd_tx_if_due(
    transport: object,
    *,
    sent: int,
    replay_t_s: float,
    now_wall: float,
) -> None:
    if not sent or not transport._sitl_cmd_debug or now_wall - transport._native_vpd_last_log_wall < 2.0:
        return
    last = transport._native_vpd_events[max(0, transport._native_vpd_cursor - 1)]
    print(
        "[sitl_transport] native VPD tx "
        f"sent={sent} replay_t={float(replay_t_s):.3f}s "
        f"event_t={last.t_replay_s:.3f}s dt_us={last.time_delta_usec} "
        f"pos_delta={last.position_delta[:3].tolist()} "
        f"conf={last.confidence:.1f}",
        flush=True,
    )
    transport._native_vpd_last_log_wall = now_wall


__all__ = ["log_native_vpd_tx_if_due"]
