"""SITL internal telemetry callbacks for ALT_HOLD diagnostics."""

from __future__ import annotations

import json
import math


def _float(payload: dict, key: str) -> float:
    try:
        return float(payload.get(key, math.nan))
    except (TypeError, ValueError):
        return math.nan


def _array_float(payload: dict, key: str, index: int) -> float:
    value = payload.get(key)
    if not isinstance(value, list) or index >= len(value):
        return math.nan
    try:
        return float(value[index])
    except (TypeError, ValueError):
        return math.nan


def on_sitl_mavlink_status(owner, msg) -> None:
    try:
        payload = json.loads(str(getattr(msg, "data", "") or "{}"))
    except json.JSONDecodeError:
        return
    owner.state.sitl_att_age_s = _float(payload, "att_age_s")
    owner.state.sitl_att_roll_rad = _float(payload, "att_roll")
    owner.state.sitl_att_pitch_rad = _float(payload, "att_pitch")
    owner.state.sitl_att_yaw_rad = _float(payload, "att_yaw")
    owner.state.sitl_att_rollspeed_rad_s = _float(payload, "att_rollspeed")
    owner.state.sitl_att_pitchspeed_rad_s = _float(payload, "att_pitchspeed")
    owner.state.sitl_att_yawspeed_rad_s = _float(payload, "att_yawspeed")

    owner.state.sitl_raw_imu_age_s = _float(payload, "raw_imu_age_s")
    owner.state.sitl_raw_imu_xgyro_rad_s = _float(payload, "raw_imu_xgyro")
    owner.state.sitl_raw_imu_ygyro_rad_s = _float(payload, "raw_imu_ygyro")
    owner.state.sitl_raw_imu_zgyro_rad_s = _float(payload, "raw_imu_zgyro")

    owner.state.sitl_scaled_imu_age_s = _float(payload, "scaled_imu_age_s")
    owner.state.sitl_scaled_imu_xgyro_rad_s = _float(payload, "scaled_imu_xgyro")
    owner.state.sitl_scaled_imu_ygyro_rad_s = _float(payload, "scaled_imu_ygyro")
    owner.state.sitl_scaled_imu_zgyro_rad_s = _float(payload, "scaled_imu_zgyro")


def on_sitl_sensor_status(owner, msg) -> None:
    try:
        payload = json.loads(str(getattr(msg, "data", "") or "{}"))
    except json.JSONDecodeError:
        return
    owner.state.live_json_timestamp_s = _float(payload, "live_json_timestamp_s")
    owner.state.live_json_gyro_x_rad_s = _array_float(payload, "live_json_gyro", 0)
    owner.state.live_json_gyro_y_rad_s = _array_float(payload, "live_json_gyro", 1)
    owner.state.live_json_gyro_z_rad_s = _array_float(payload, "live_json_gyro", 2)
    owner.state.live_json_accel_x_mps2 = _array_float(payload, "live_json_accel_body", 0)
    owner.state.live_json_accel_y_mps2 = _array_float(payload, "live_json_accel_body", 1)
    owner.state.live_json_accel_z_mps2 = _array_float(payload, "live_json_accel_body", 2)
    owner.state.live_json_roll_rad = _array_float(payload, "live_json_attitude", 0)
    owner.state.live_json_pitch_rad = _array_float(payload, "live_json_attitude", 1)
    owner.state.live_json_yaw_rad = _array_float(payload, "live_json_attitude", 2)


__all__ = ["on_sitl_mavlink_status", "on_sitl_sensor_status"]
