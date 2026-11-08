"""Select causal recorded gyro inputs without fitting a bias or time shift."""

from __future__ import annotations

import numpy as np


def gyro_frd_from_recorded_raw_imu(
    raw_history: np.ndarray, t_s: float
) -> tuple[np.ndarray | None, float | None]:
    """Select a received calibrated INS gyro without another drift correction.

    Args:
        raw_history: Receipt-ordered ROS data_raw rows. Receipt time [s] is
            column 0; FLU angular velocity [rad/s] is in columns 6:9.
        t_s: Current replay receipt time [s].

    Returns:
        Held FRD gyro [rad/s] and its original receipt time [s]. Both are None
        before the first measurement. No future value or AHRS fallback is used.
    """
    if len(raw_history) == 0:
        return None, None
    index = np.searchsorted(raw_history[:, 0], t_s, side="right") - 1
    if index < 0:
        return None, None
    return np.asarray(raw_history[index, 6:9], dtype=float) * [1, -1, -1], float(raw_history[index, 0])


def gyro_frd_from_recorded_imu(
    imu_row: np.ndarray,
    ahrs_history: np.ndarray,
    *,
    gyro_source: str,
    max_ahrs_age_s: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, float | None]:
    """Recover the pre-EKF gyro from observed attitude rates and AHRS drift.

    ArduSub 4.1.2 exports ATTITUDE rates as INS gyro plus AHRS gyro drift.
    RAW_IMU exports the calibrated INS gyro before that EKF drift correction.

    Args:
        imu_row: Native ROS IMU row; receipt time [s] at index 0 and FLU
            angular velocity [rad/s] at indices 6:9.
        ahrs_history: Receipt-ordered native AHRS rows; receipt time [s] at
            index 0 and FRD gyro drift [rad/s] at indices 2:5.
        gyro_source: ``attitude`` preserves the prior diagnostic input;
            ``attitude_minus_ahrs`` inverts the recorded gyro drift correction.
        max_ahrs_age_s: Maximum age of a preceding AHRS sample [s].

    Returns:
        FRD gyro input [rad/s], removed FRD drift [rad/s], and AHRS receipt
        time [s], or None when no supported preceding observation exists.
        Unsupported history preserves the prior input; it is never filled
        from a future AHRS sample or a fitted calibration value.
    """
    if gyro_source not in {"attitude", "attitude_minus_ahrs"}:
        raise ValueError(f"Unsupported recorded gyro source: {gyro_source}")
    gyro = np.asarray(imu_row[6:9], dtype=float) * [1, -1, -1]
    drift = np.zeros(3)
    if gyro_source == "attitude" or len(ahrs_history) == 0:
        return gyro, drift, None
    index = np.searchsorted(ahrs_history[:, 0], imu_row[0], side="right") - 1
    if index >= 0 and 0 <= imu_row[0] - ahrs_history[index, 0] <= max_ahrs_age_s:
        drift = np.asarray(ahrs_history[index, 2:5], dtype=float).copy()
        return gyro - drift, drift, float(ahrs_history[index, 0])
    return gyro, drift, None
