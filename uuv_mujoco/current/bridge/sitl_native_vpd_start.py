"""Native VPD replay start/cursor policy."""

from __future__ import annotations


def start_native_vpd_replay_if_needed(transport: object, *, replay_t_s: float) -> None:
    if transport._native_vpd_started:
        return
    start_cutoff = float(replay_t_s) - float(transport._native_vpd_start_tolerance_s)
    skipped = 0
    while (
        transport._native_vpd_cursor < len(transport._native_vpd_events)
        and transport._native_vpd_events[transport._native_vpd_cursor].t_replay_s < start_cutoff
    ):
        transport._native_vpd_cursor += 1
        skipped += 1
    transport._native_vpd_started = True
    print(
        "[sitl_transport] native VPD replay started at "
        f"replay_t={float(replay_t_s):.3f}s cursor={transport._native_vpd_cursor} "
        f"skipped={skipped}",
        flush=True,
    )


__all__ = ["start_native_vpd_replay_if_needed"]
