"""Status snapshot and state properties for SitlTransport."""

from __future__ import annotations

import time

from bridge.sitl_status import build_mavlink_telemetry_status


def mavlink_telemetry_status(self) -> dict[str, object]:
    """Latest passive ArduPilot MAVLink telemetry observed by the SITL link."""
    now_wall = time.monotonic()
    return build_mavlink_telemetry_status(
        base_status=self._mavlink_telemetry_observer.status_snapshot(now_wall),
        now_wall=now_wall,
        command_endpoint=str(self._sitl_cmd_mavlink_endpoint),
        command_hb_wall=float(self._sitl_cmd_mav_last_hb_wall),
        servo_hb_wall=float(self._sitl_mav_last_hb_wall),
        command_servo_msg_count=int(getattr(self, "_sitl_cmd_servo_msg_count", 0)),
        command_servo_last_msg_wall=float(getattr(self, "_sitl_cmd_servo_last_msg_wall", -1.0)),
        servo_link_servo_msg_count=int(getattr(self, "_sitl_servo_link_servo_msg_count", 0)),
        servo_link_servo_last_msg_wall=float(getattr(self, "_sitl_servo_link_servo_last_msg_wall", -1.0)),
        rc_override_ready=bool(self.rc_override_ready),
        vehicle_armed=bool(self._sitl_vehicle_armed),
        vehicle_mode=str(self._sitl_vehicle_mode or ""),
        auto_ready_enabled=bool(self._sitl_auto_ready_enabled),
        auto_ready_mode=str(self._sitl_auto_ready_mode),
        auto_ready_state=str(self._sitl_auto_ready_state),
        auto_ready_done_wall=float(self._sitl_auto_ready_done_wall),
        extnav_enabled=bool(self._sitl_extnav_enabled),
        extnav_required=bool(self._sitl_extnav_required),
        extnav_last_send_wall=float(self._sitl_extnav_last_send_wall),
        extnav_last_rate_hz=float(self._sitl_extnav_last_rate_hz),
        extnav_min_tx_hz=float(self._sitl_extnav_min_tx_hz),
        extnav_max_stale_s=float(self._sitl_extnav_max_stale_s),
        extnav_scheduler=str(self._sitl_extnav_scheduler),
        extnav_start_wall=float(self._sitl_extnav_start_wall),
        extnav_grace_s=float(self._sitl_extnav_grace_s),
        extnav_fault=str(self._sitl_extnav_fault),
    )


@property
def vehicle_armed(self) -> bool:
    return bool(self._sitl_vehicle_armed)


@property
def vehicle_mode(self) -> str:
    return str(self._sitl_vehicle_mode or "")


@property
def last_rc_override_values(self) -> list[int]:
    return list(self._sitl_last_rc_override_values[:18])


@property
def last_rc_override_age_s(self) -> float:
    if self._sitl_last_rc_override_values_wall <= 0.0:
        return float("inf")
    return float(max(0.0, time.monotonic() - self._sitl_last_rc_override_values_wall))


__all__ = [
    "last_rc_override_age_s",
    "last_rc_override_values",
    "mavlink_telemetry_status",
    "vehicle_armed",
    "vehicle_mode",
]
