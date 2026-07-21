"""Neutral RC keepalive policy for SitlTransport."""

from __future__ import annotations


def _send_neutral_rc_keepalive(self, now_wall: float) -> None:
    # Auto-ready sends neutral RC only while arming or changing mode via
    # _send_auto_ready_neutral_rc().  After READY, implicit neutral
    # keepalive must not compete with operator /mavros/rc/override input.
    if not self._sitl_neutral_rc_keepalive or not self._sitl_vehicle_armed:
        return
    if now_wall - self._sitl_last_neutral_rc_keepalive_wall < self._sitl_neutral_rc_keepalive_interval_s:
        return
    if (
        self._sitl_last_external_rc_override_wall > 0.0
        and now_wall - self._sitl_last_external_rc_override_wall < self._sitl_neutral_rc_keepalive_holdoff_s
    ):
        return
    mav = self._mav_for_commands()
    target = self._resolve_mav_target(mav)
    if mav is None or target is None:
        return
    target_sys, target_comp = target
    try:
        self._send_gcs_heartbeat(force=True, mav=mav)
        values = self._neutral_rc_values()
        self._send_rc_channels_override(mav, target_sys, target_comp, values)
        self._sitl_last_neutral_rc_keepalive_wall = now_wall
    except Exception:
        return


__all__ = ["_send_neutral_rc_keepalive"]
