"""MANUAL_CONTROL helpers for SitlTransport."""

from __future__ import annotations

from bridge.sitl_manual_control_frame import (
    build_manual_control_frame,
    prime_manual_control_if_needed,
    send_manual_control_frame,
)
from bridge.sitl_manual_control_logging import mark_manual_control_sent


def send_manual_control(
    self,
    *,
    x: float,
    y: float,
    z: float,
    r: float,
    buttons: int = 0,
) -> bool:
    """Forward MAVROS-style normalized MANUAL_CONTROL to ArduSub SITL."""
    mav = self._mav_for_commands()
    if mav is None:
        return False
    target = self._resolve_mav_target(mav)
    if target is None:
        return False
    target_sys, _target_comp = target

    try:
        self._send_gcs_heartbeat(force=True, mav=mav)
        frame = build_manual_control_frame(
            x=x,
            y=y,
            z=z,
            r=r,
            buttons=buttons,
        )
        prime_manual_control_if_needed(self, mav, target_sys=int(target_sys), frame=frame)
        send_manual_control_frame(mav, target_sys=int(target_sys), frame=frame)
        mark_manual_control_sent(self, target_sys=int(target_sys), frame=frame)
        return True
    except Exception as exc:
        print(f"[sitl_transport] MANUAL_CONTROL send failed: {exc}", flush=True)
        return False


__all__ = [
    "build_manual_control_frame",
    "mark_manual_control_sent",
    "prime_manual_control_if_needed",
    "send_manual_control",
    "send_manual_control_frame",
]
