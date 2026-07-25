"""Neutral-RC priming helpers for SITL auto-ready."""

from __future__ import annotations

from sim.contracts import neutral_rc_override_frame


def _neutral_rc_values(self) -> list[int]:
    return neutral_rc_override_frame(center_pwm=int(self._sitl_rc_neutral_pwm))


def _send_auto_ready_neutral_rc(self, now_wall: float) -> None:
    if now_wall - self._sitl_auto_ready_last_neutral_wall < 0.20:
        return
    sent_any = False
    values = self._neutral_rc_values()
    for mav in self._mavs_for_arm_mode_commands():
        target = self._resolve_mav_target(mav)
        if target is None:
            continue
        target_sys, target_comp = target
        try:
            self._send_gcs_heartbeat(force=True, mav=mav)
            self._send_rc_channels_override(mav, target_sys, target_comp, values)
            sent_any = True
        except Exception:
            continue
    if sent_any:
        self._sitl_auto_ready_last_neutral_wall = now_wall


__all__ = ["_neutral_rc_values", "_send_auto_ready_neutral_rc"]
