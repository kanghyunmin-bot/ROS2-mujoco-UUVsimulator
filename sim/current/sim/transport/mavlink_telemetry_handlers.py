"""Target-source MAVLink telemetry handlers."""

from __future__ import annotations

from .mavlink_telemetry_storage import store_fields, store_rc_channels


ATTITUDE_FIELDS = ("time_boot_ms", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed")
LOCAL_POSITION_FIELDS = ("time_boot_ms", "x", "y", "z", "vx", "vy", "vz")
SCALED_PRESSURE_FIELDS = ("time_boot_ms", "press_abs", "press_diff", "temperature", "temperature_press_diff")
EKF_STATUS_FIELDS = (
    "flags",
    "velocity_variance",
    "pos_horiz_variance",
    "pos_vert_variance",
    "compass_variance",
    "terrain_alt_variance",
    "airspeed_variance",
)
VFR_HUD_FIELDS = ("airspeed", "groundspeed", "heading", "throttle", "alt", "climb")
IMU_FIELDS = (
    "time_usec",
    "time_boot_ms",
    "xacc",
    "yacc",
    "zacc",
    "xgyro",
    "ygyro",
    "zgyro",
    "xmag",
    "ymag",
    "zmag",
)
STATUSTEXT_FIELDS = ("severity", "text", "id", "chunk_seq")

PRESSURE_PREFIX_BY_TYPE = {
    "SCALED_PRESSURE": "spress1",
    "SCALED_PRESSURE2": "spress2",
    "SCALED_PRESSURE3": "spress3",
}
IMU_PREFIX_BY_TYPE = {
    "RAW_IMU": "raw_imu",
    "SCALED_IMU": "scaled_imu",
}


def handle_target_mavlink_telemetry(
    status_data: dict[str, object],
    msg_type: str,
    data: dict[str, object],
    now_wall: float,
) -> None:
    if msg_type == "ATTITUDE":
        store_fields(status_data, "att", data, now_wall, ATTITUDE_FIELDS)
    elif msg_type == "LOCAL_POSITION_NED":
        store_fields(status_data, "lpos", data, now_wall, LOCAL_POSITION_FIELDS)
    elif msg_type in PRESSURE_PREFIX_BY_TYPE:
        store_fields(status_data, PRESSURE_PREFIX_BY_TYPE[msg_type], data, now_wall, SCALED_PRESSURE_FIELDS)
    elif msg_type == "RC_CHANNELS":
        store_rc_channels(status_data, data, now_wall)
    elif msg_type == "EKF_STATUS_REPORT":
        store_fields(status_data, "ekf", data, now_wall, EKF_STATUS_FIELDS)
    elif msg_type == "VFR_HUD":
        store_fields(status_data, "vfr", data, now_wall, VFR_HUD_FIELDS)
    elif msg_type in IMU_PREFIX_BY_TYPE:
        store_fields(status_data, IMU_PREFIX_BY_TYPE[msg_type], data, now_wall, IMU_FIELDS)
    elif msg_type == "STATUSTEXT":
        store_fields(status_data, "statustext", data, now_wall, STATUSTEXT_FIELDS)


__all__ = [
    "ATTITUDE_FIELDS",
    "EKF_STATUS_FIELDS",
    "IMU_FIELDS",
    "IMU_PREFIX_BY_TYPE",
    "LOCAL_POSITION_FIELDS",
    "PRESSURE_PREFIX_BY_TYPE",
    "SCALED_PRESSURE_FIELDS",
    "STATUSTEXT_FIELDS",
    "VFR_HUD_FIELDS",
    "handle_target_mavlink_telemetry",
]
