"""Resolve one recorded arming state without fitting sensor poses or dynamics."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def sample_at_time(
    rows: np.ndarray, time_s: float, maximum_gap_s: float, *, quaternion: bool = False
) -> tuple[np.ndarray, list[float]]:
    """Sample received observations [s] within bounded support; retain source brackets."""
    rows = np.asarray(rows, dtype=float)
    if len(rows) < 1 or not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("Expected finite, ordered observations")
    right = int(np.searchsorted(rows[:, 0], time_s, side="left"))
    left = max(0, right - 1)
    right = min(right, len(rows) - 1)
    if rows[right, 0] == time_s:
        left = right
    if right == left:
        if abs(rows[left, 0] - time_s) > maximum_gap_s:
            raise ValueError("No nearby observation at arming")
        value = rows[left, 1:].copy()
    else:
        if rows[right, 0] - rows[left, 0] > maximum_gap_s:
            raise ValueError("Observation gap spans arming")
        if quaternion:
            value = Slerp(rows[[left, right], 0], Rotation.from_quat(rows[[left, right], 1:]))(
                [time_s]
            ).as_quat()[0]
        else:
            alpha = (time_s - rows[left, 0]) / (rows[right, 0] - rows[left, 0])
            value = rows[left, 1:] * (1 - alpha) + rows[right, 1:] * alpha
    if quaternion:
        value = Rotation.from_quat(value).as_quat()
    return value, rows[[left, right], 0].tolist()


def build_armed_initial_state(
    states: list[list],
    imu: np.ndarray,
    pressure: np.ndarray,
    base_from_imu: np.ndarray,
    pressure_position_base_m: np.ndarray,
    surface_pressure_pa: float,
) -> dict:
    """Resolve base pose [m, rad] and angular rate [rad/s] at first observed arming.

    Pressure [Pa] is converted with the frozen replay water density and gravity.
    Message orientation is world-from-IMU; extrinsics are base-from-IMU. Linear
    velocity remains the previous zero assumption, without using unverified DVL.
    """
    observed = next((row for row in states if row[1] and row[2]), None)
    if observed is None:
        raise ValueError("No connected armed observation")
    time_s = float(observed[0])
    quat, imu_bracket = sample_at_time(imu[:, :5], time_s, .25, quaternion=True)
    gyro, gyro_bracket = sample_at_time(imu[:, [0, 5, 6, 7]], time_s, .25)
    p, pressure_bracket = sample_at_time(pressure, time_s, 2.1)
    world_from_imu = Rotation.from_quat(quat)
    base_from_imu = Rotation.from_matrix(np.asarray(base_from_imu)[:3, :3])
    world_from_base = world_from_imu * base_from_imu.inv()
    base_gyro = base_from_imu.apply(gyro)
    sensor_depth_m = (float(p[0]) - surface_pressure_pa) / 9806.65
    base_depth_m = sensor_depth_m + world_from_base.apply(pressure_position_base_m)[2]
    return {
        "schema": "uuv.armed_initial_state.v1",
        "arm_bag_time_s": time_s,
        "mode": observed[3],
        "pressure_depth_m": sensor_depth_m,
        "base_depth_m": float(base_depth_m),
        "quaternion_xyzw": world_from_base.as_quat().tolist(),
        "rpy_rad": world_from_base.as_euler("xyz").tolist(),
        "angular_velocity_body_radps": base_gyro.tolist(),
        "linear_velocity_body_mps": [0., 0., 0.],
        "surface_pressure_pa": float(surface_pressure_pa),
        "imu_bracket_s": imu_bracket,
        "gyro_bracket_s": gyro_bracket,
        "pressure_bracket_s": pressure_bracket,
        "pressure_coarse_support": pressure_bracket[1] - pressure_bracket[0] > .75,
        "scope": (
            "First connected armed observation; one initialization per bag. "
            "No later state injection, fitted delay, DVL velocity or physics coefficient change."
        ),
    }
