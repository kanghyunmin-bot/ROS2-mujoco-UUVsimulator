"""DVL velocity extraction for real-start rows."""

from __future__ import annotations

from real_start_velocity_vectors import finite_row_vector


def dvl_velocity_from_row(row: dict[str, str], source: str) -> tuple[float, float, float, str] | None:
    dvl_vel = finite_row_vector(row, ("dvl_twist_x", "dvl_twist_y", "dvl_twist_z"))
    if dvl_vel is not None:
        return float(dvl_vel[0]), float(dvl_vel[1]), float(dvl_vel[2]), source
    return None


__all__ = ["dvl_velocity_from_row"]
