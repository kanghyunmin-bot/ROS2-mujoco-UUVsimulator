"""ArduSub JSON sensor payload construction."""

from __future__ import annotations

import numpy as np

from bridge.sitl_types import VerticalEstimate


def _payload_from_state(
    self,
    sensor_time_s: float,
    gyro: np.ndarray,
    acc: np.ndarray,
    vertical_est: VerticalEstimate,
    quat: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
    rangefinder_distance_m: float | None,
) -> dict[str, object]:
    json_position = np.asarray(vertical_est.pos_ned, dtype=np.float64).copy()
    json_velocity = np.asarray(vertical_est.vel_ned, dtype=np.float64).copy()
    # ArduSub's JSON backend stores position.z as NED down. AP_Baro_SITL
    # then converts location altitude back into underwater pressure, so
    # Bar30 depth must stay positive-down here.
    json_position[2] = float(vertical_est.depth_m)
    json_velocity[2] = float(vertical_est.vel_ned[2])
    payload: dict[str, object] = {
        "timestamp": float(sensor_time_s),
        "altitude": float(vertical_est.alt_m),
        "imu": {
            "gyro": [float(x) for x in gyro],
            "accel_body": [float(x) for x in acc],
        },
        "position": [float(x) for x in json_position],
        "velocity": [float(x) for x in json_velocity],
        "attitude": [float(roll), float(pitch), float(yaw)],
        "quaternion": [float(x) for x in quat],
        "no_time_sync": True,
        "no_lockstep": True,
    }
    if rangefinder_distance_m is not None and np.isfinite(float(rangefinder_distance_m)):
        payload["rng_1"] = float(rangefinder_distance_m)
    return payload


__all__ = ["_payload_from_state"]
