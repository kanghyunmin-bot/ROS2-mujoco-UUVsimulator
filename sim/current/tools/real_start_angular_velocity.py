"""Angular velocity extraction for real-start rows."""

from __future__ import annotations

from real_start_velocity_vectors import finite_row_vector


def angular_velocity_from_row(row: dict[str, str]) -> tuple[float, float, float, str]:
    gyro = finite_row_vector(row, ("imu_gyro_x", "imu_gyro_y", "imu_gyro_z"))
    if gyro is not None:
        return float(gyro[0]), float(gyro[1]), float(gyro[2]), "imu_gyro_body"
    return 0.0, 0.0, 0.0, "fallback_zero"


__all__ = ["angular_velocity_from_row"]
