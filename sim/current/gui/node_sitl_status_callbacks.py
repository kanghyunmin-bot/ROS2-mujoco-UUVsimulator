"""SITL and real-start status callbacks for UuvGuiNode."""

from __future__ import annotations

import json
import math


def _on_sitl_mavlink_telemetry_status(self, msg: String) -> None:
    self._touch("sitl_mavlink_status")
    try:
        payload = json.loads(str(msg.data))
    except json.JSONDecodeError:
        payload = {}
    with self._lock:
        self._snapshot.sitl_mavlink_status = dict(payload)
        self._snapshot.sitl_mavlink_active = bool(payload.get("active", False))
        self._snapshot.sitl_mavlink_endpoint = str(payload.get("mavlink_endpoint", "") or "")
        self._snapshot.sitl_mavlink_heartbeat_age_s = self._payload_float(
            payload,
            "heartbeat_age_s",
        )
        self._snapshot.sitl_mavlink_command_heartbeat_age_s = self._payload_float(
            payload,
            "command_heartbeat_age_s",
        )
        self._snapshot.sitl_mavlink_rc_channels_age_s = self._payload_float(
            payload,
            "rc_channels_age_s",
        )
        self._snapshot.sitl_mavlink_rc_override_ready = bool(payload.get("rc_override_ready", False))
        self._snapshot.sitl_extnav_required = bool(payload.get("extnav_required", False))
        self._snapshot.sitl_extnav_ready = bool(payload.get("extnav_ready", True))
        self._snapshot.sitl_extnav_last_rate_hz = self._payload_float(
            payload,
            "extnav_last_rate_hz",
            default=0.0,
        )


def _on_real_start_status(self, msg: String) -> None:
    self._touch("real_start")
    try:
        payload = json.loads(str(msg.data))
    except json.JSONDecodeError:
        payload = {}
    with self._lock:
        self._snapshot.real_start_required = bool(payload.get("required", False))
        self._snapshot.real_start_ok = bool(payload.get("ok", not self._snapshot.real_start_required))
        self._snapshot.real_start_released = bool(payload.get("released", False))
        self._snapshot.real_start_status = str(payload.get("status", "unknown"))
        self._snapshot.real_start_depth_error_m = float(payload.get("depth_error_m", math.nan))
        self._snapshot.real_start_attitude_error_rad = float(payload.get("attitude_error_rad", math.nan))
        self._snapshot.real_start_velocity_error_mps = float(payload.get("velocity_error_mps", math.nan))
        if self._snapshot.real_start_released:
            self._initial_depth_release_pending = False
            self._initial_depth_release_in_flight = False
            self._initial_depth_release_reason = ""


__all__ = ["_on_real_start_status", "_on_sitl_mavlink_telemetry_status"]
