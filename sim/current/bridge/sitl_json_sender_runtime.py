"""Top-level ArduSub JSON sensor packet send helper."""

from __future__ import annotations

from bridge.sitl_types import VerticalEstimate

from .sitl_json_sender_diagnostics import (
    log_sitl_sensor_sample_due,
    log_sitl_send_failure_due,
    log_sitl_send_status_due,
)
from .sitl_json_sender_io import current_sitl_json_target, send_sitl_json_bytes
from .sitl_json_sender_validation import (
    sitl_json_payload_is_finite,
    warn_nonfinite_sitl_json_payload_once,
)


def _send_sitl_json_payload(
    self,
    payload: dict[str, object],
    *,
    now_wall: float,
    vertical_est: VerticalEstimate,
    pressure_pa: float | None,
) -> None:
    if not sitl_json_payload_is_finite(payload):
        warn_nonfinite_sitl_json_payload_once(self)
        return

    try:
        self._sitl_last_json_payload_status = {
            "live_json_timestamp_s": float(payload["timestamp"]),
            "live_json_gyro": [float(x) for x in payload["imu"]["gyro"]],
            "live_json_accel_body": [float(x) for x in payload["imu"]["accel_body"]],
            "live_json_position": [float(x) for x in payload["position"]],
            "live_json_velocity": [float(x) for x in payload["velocity"]],
            "live_json_attitude": [float(x) for x in payload["attitude"]],
            "live_json_quaternion": [float(x) for x in payload["quaternion"]],
            "live_json_depth_m": float(vertical_est.depth_m),
            "live_json_pressure_pa": None if pressure_pa is None else float(pressure_pa),
        }
    except Exception:
        self._sitl_last_json_payload_status = {}

    log_sitl_sensor_sample_due(
        self,
        payload,
        now_wall=now_wall,
        vertical_est=vertical_est,
        pressure_pa=pressure_pa,
    )

    target = current_sitl_json_target(self)
    try:
        prev_target = self._sitl_send_target
        sent, target = send_sitl_json_bytes(self, payload, target)
        log_sitl_send_status_due(
            self,
            now_wall=now_wall,
            target=target,
            sent=sent,
            prev_target=prev_target,
        )
    except Exception as exc:
        log_sitl_send_failure_due(self, now_wall=now_wall, target=target, exc=exc)


__all__ = ["_send_sitl_json_payload"]
