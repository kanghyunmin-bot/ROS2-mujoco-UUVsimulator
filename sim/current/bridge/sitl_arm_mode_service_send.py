"""Broadcast helpers for pending SitlTransport arm/mode retries."""

from __future__ import annotations


def send_pending_arm_to_any_link(self, target_arm: bool) -> bool:
    sent_any = False
    send_initial_neutral = bool(target_arm) and not bool(
        getattr(self, "_sitl_pending_arm_neutral_sent", False)
    )
    neutral_sent = False
    for mav in self._mavs_for_arm_mode_commands():
        target = self._resolve_mav_target(mav)
        if target is None:
            continue
        target_sys, target_comp = target
        self._send_gcs_heartbeat(force=True, mav=mav)
        # Native SITL can acknowledge normal arm/disarm commands without
        # reliably changing HEARTBEAT armed state. Use ArduPilot's force
        # magic in both directions so GUI/QGC smoke commands are deterministic.
        sent = self._send_arm_disarm_mavlink(mav, target_sys, target_comp, target_arm, force=True)
        if send_initial_neutral:
            send_arm_neutral_rc(self, mav, target_sys, target_comp)
            neutral_sent = True
        sent_any = bool(sent) or sent_any
        if self._sitl_cmd_debug and not sent:
            print(
                f"[sitl_transport] arm/disarm send failed target={target_sys}:{target_comp} "
                f"armed={bool(target_arm)}",
                flush=True,
            )
    if neutral_sent:
        self._sitl_pending_arm_neutral_sent = True
    return sent_any


def send_pending_mode_to_any_link(self, mode: str) -> bool:
    sent_any = False
    for mav in self._mavs_for_arm_mode_commands():
        target = self._resolve_mav_target(mav)
        if target is None:
            continue
        target_sys, target_comp = target
        self._send_gcs_heartbeat(force=True, mav=mav)
        if self._send_set_mode_mavlink(mav, target_sys, target_comp, mode):
            sent_any = True
    return sent_any


def send_arm_neutral_rc(self, mav, target_sys: int, target_comp: int) -> None:
    try:
        neutral_values = self._neutral_rc_values()
        self._send_rc_channels_override(mav, target_sys, target_comp, neutral_values)
    except Exception:
        pass


__all__ = [
    "send_arm_neutral_rc",
    "send_pending_arm_to_any_link",
    "send_pending_mode_to_any_link",
]
