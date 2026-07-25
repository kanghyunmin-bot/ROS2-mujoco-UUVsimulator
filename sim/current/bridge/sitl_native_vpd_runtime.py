"""Native VISION_POSITION_DELTA replay runtime for SITL ExternalNav."""

from __future__ import annotations

from bridge.sitl_native_vpd_debug import log_native_vpd_tx_if_due
from bridge.sitl_native_vpd_rate import update_native_vpd_rate_window
from bridge.sitl_native_vpd_send import send_due_native_vpd_events
from bridge.sitl_native_vpd_start import start_native_vpd_replay_if_needed


def _send_native_vision_delta_due(self, mav, now_wall: float) -> bool:
    """Replay recorded VPD events due at the current sensor-replay time.

    Returns True when native VPD mode is active. In that case callers must not
    synthesize another VPD stream from interpolated pose, otherwise the ArduSub
    EKF sees a different input contract than the isolated parity harness and
    the real vehicle log.
    """
    if not self._native_vpd_events:
        return False
    if mav is None:
        return True
    if not self._sensor_replay_ready or self._sensor_replay_current_t_s is None:
        return True

    replay_t_s = float(self._sensor_replay_current_t_s)
    start_native_vpd_replay_if_needed(self, replay_t_s=replay_t_s)
    sent = send_due_native_vpd_events(self, mav, replay_t_s=replay_t_s, now_wall=now_wall)
    update_native_vpd_rate_window(self, now_wall=now_wall)
    log_native_vpd_tx_if_due(self, sent=sent, replay_t_s=replay_t_s, now_wall=now_wall)
    return True


__all__ = ["_send_native_vision_delta_due"]
