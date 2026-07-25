"""SITL servo-output telemetry mirror for MAVROS RCOut."""

from __future__ import annotations

from .ros2_rcout_message import build_sitl_servo_rcout_message
from .ros2_rcout_publish import publish_rcout_event_if_enabled
from .ros2_rcout_stamp import sensor_replay_real_stamp as _sensor_replay_real_stamp


def _on_sitl_servo_output_for_ros(self, pwm_values: list[int]) -> None:
    if not self._mavros_surface_enabled or self.RCOut is None:
        self._mavros_last_rc_out = None
        return
    try:
        rc_out = build_sitl_servo_rcout_message(self, pwm_values)
        self._mavros_last_rc_out = rc_out
        publish_rcout_event_if_enabled(self, rc_out)
    except Exception:
        self._mavros_last_rc_out = None


__all__ = [
    "_on_sitl_servo_output_for_ros",
    "_sensor_replay_real_stamp",
    "build_sitl_servo_rcout_message",
    "publish_rcout_event_if_enabled",
]
