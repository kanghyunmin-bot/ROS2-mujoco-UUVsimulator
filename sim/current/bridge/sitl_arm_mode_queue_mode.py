"""Mode queue helper for SitlTransport."""

from __future__ import annotations

import time


def queue_set_mode_impl(transport, mode: str) -> bool:
    mode_text = str(mode or "").strip()
    if not mode_text:
        return False
    try:
        transport._mode_id_for_text(mode_text)
    except Exception as exc:
        print(f"[sitl_transport] set_mode queue rejected for {mode_text!r}: {exc}", flush=True)
        return False
    now_wall = time.monotonic()
    if str(transport._sitl_vehicle_mode or "").upper() == mode_text.upper():
        return True
    if transport._sitl_pending_mode and str(transport._sitl_pending_mode).upper() == mode_text.upper():
        transport._service_pending_mode_command(now_wall)
        return True
    transport._sitl_pending_mode = mode_text
    transport._sitl_pending_mode_start_wall = now_wall
    transport._sitl_pending_mode_last_send_wall = 0.0
    print(f"[sitl_transport] set_mode command queued: mode={mode_text!r}", flush=True)
    transport._service_pending_mode_command(now_wall)
    return True


__all__ = ["queue_set_mode_impl"]
