"""RC override forwarding path for SitlTransport."""

from __future__ import annotations

import time


def send_rc_override(self, pwm_values: list[int]) -> bool:
    """Forward MAVROS-style RC override to ArduSub SITL over MAVLink.

    RC override is pilot input, so keep the immediate dist-style 8-channel
    frame, but send it over the active command link. The dedicated serial4
    command link is the startup-readiness contract.
    """
    mav = self._mav_for_commands()
    if mav is None:
        self._warn_rc_override_not_forwarded("MAVLink command endpoint is disabled or unavailable")
        return False
    target = self._resolve_mav_target(mav)
    if target is None:
        self._warn_rc_override_not_forwarded("target system/component not resolved yet; waiting for ArduSub heartbeat")
        return False
    target_sys, target_comp = target
    values = _dist_style_primary_rc_frame(pwm_values)
    try:
        sent_channels = self._send_rc_channels_override(mav, target_sys, target_comp, values)
        if sent_channels <= 0:
            self._warn_rc_override_not_forwarded("MAVLink command link is not ready for RC override")
            return False
        now = time.monotonic()
        _record_rc_override_success(self, values, now)
        _log_rc_override_forwarded(self, target_sys, target_comp, sent_channels, values, now)
        return True
    except Exception as exc:
        print(f"[sitl_transport] RC override send failed: {exc}", flush=True)
        return False


def _dist_style_primary_rc_frame(pwm_values: list[int]) -> list[int]:
    values = [int(v) for v in list(pwm_values[:8])]
    if len(values) < 8:
        values.extend([65535] * (8 - len(values)))
    return values


def _record_rc_override_success(self, values: list[int], now_wall: float) -> None:
    self._sitl_last_external_rc_override_wall = now_wall
    self._sitl_last_rc_override_values = [int(v) for v in values[:8]]
    self._sitl_last_rc_override_values_wall = now_wall
    if not self._sensor_replay_rc_seen:
        self._sensor_replay_immediate_last_frame_count = None
    self._sensor_replay_rc_seen = True


def _log_rc_override_forwarded(
    self,
    target_sys: int,
    target_comp: int,
    sent_channels: int,
    values: list[int],
    now_wall: float,
) -> None:
    if not self._sitl_cmd_debug:
        return
    if now_wall - self._sitl_last_rc_override_log_wall <= 0.2:
        return
    print(
        f"[sitl_transport] RC override forwarded to ArduSub "
        f"target={target_sys}:{target_comp} channels={sent_channels} "
        f"pwm[1..8]={tuple(int(v) for v in values[:8])} "
        f"pwm[9..18]={tuple(int(v) for v in values[8:18])}",
        flush=True,
    )
    self._sitl_last_rc_override_log_wall = now_wall


__all__ = ["send_rc_override"]
