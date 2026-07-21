"""Native VPD due-event sender."""

from __future__ import annotations


def send_due_native_vpd_events(
    transport: object,
    mav: object,
    *,
    replay_t_s: float,
    now_wall: float,
) -> int:
    sent = 0
    while (
        transport._native_vpd_cursor < len(transport._native_vpd_events)
        and transport._native_vpd_events[transport._native_vpd_cursor].t_replay_s
        <= float(replay_t_s) + 1.0e-9
    ):
        event = transport._native_vpd_events[transport._native_vpd_cursor]
        try:
            mav.mav.vision_position_delta_send(
                int(event.time_usec),
                int(event.time_delta_usec),
                [float(v) for v in event.angle_delta[:3]],
                [float(v) for v in event.position_delta[:3]],
                float(event.confidence),
            )
        except Exception as exc:
            if now_wall - transport._sitl_extnav_send_failed_wall > 2.0:
                print(f"[sitl_transport] native VPD send failed: {exc}", flush=True)
                transport._sitl_extnav_send_failed_wall = now_wall
            break
        transport._native_vpd_last_replay_t_s = float(event.t_replay_s)
        transport._sitl_extnav_last_send_sim_t = float(replay_t_s)
        transport._sitl_extnav_last_send_wall = now_wall
        transport._sitl_extnav_tx_window_count += 1
        transport._native_vpd_cursor += 1
        sent += 1
    return sent


__all__ = ["send_due_native_vpd_events"]
