"""MAVLink telemetry extraction helpers for SITL transport."""

from __future__ import annotations

SERVO_OUTPUT_RAW_TYPE = "SERVO_OUTPUT_RAW"
AP_TELEMETRY_TYPES = (
    "HEARTBEAT",
    "SERVO_OUTPUT_RAW",
    "ATTITUDE",
    "LOCAL_POSITION_NED",
    "SCALED_PRESSURE",
    "SCALED_PRESSURE2",
    "SCALED_PRESSURE3",
    "RC_CHANNELS",
    "EKF_STATUS_REPORT",
    "VFR_HUD",
    "RAW_IMU",
    "SCALED_IMU",
    "STATUSTEXT",
)


def message_type(message: object) -> str:
    try:
        return str(message.get_type())
    except Exception:
        return ""


def servo_output_raw_pwm_values(message: object, *, channels: int = 8) -> list[int] | None:
    """Extract PWM channels from a MAVLink SERVO_OUTPUT_RAW-like message."""
    try:
        data = message.to_dict()
    except Exception:
        return None
    try:
        return [int(data.get(f"servo{i}_raw", 0)) for i in range(1, int(channels) + 1)]
    except Exception:
        return None


__all__ = [
    "SERVO_OUTPUT_RAW_TYPE",
    "AP_TELEMETRY_TYPES",
    "message_type",
    "servo_output_raw_pwm_values",
]
