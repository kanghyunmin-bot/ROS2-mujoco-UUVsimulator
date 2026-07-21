"""Motion and sensor field helpers for axis RC samples."""

from __future__ import annotations

from typing import Any

from axis_rc_metrics import quat_to_rpy_rad


def add_imu(sample: dict[str, Any], imu: Any) -> None:
    if imu is None:
        return
    q = imu.orientation
    roll, pitch, yaw = quat_to_rpy_rad(float(q.w), float(q.x), float(q.y), float(q.z))
    sample.update(
        {
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw,
            "gyro_x": float(imu.angular_velocity.x),
            "gyro_y": float(imu.angular_velocity.y),
            "gyro_z": float(imu.angular_velocity.z),
            "acc_x": float(imu.linear_acceleration.x),
            "acc_y": float(imu.linear_acceleration.y),
            "acc_z": float(imu.linear_acceleration.z),
        }
    )


def add_dvl(sample: dict[str, Any], dvl_twist: Any) -> None:
    if dvl_twist is None:
        return
    twist = dvl_twist.twist
    v = getattr(twist, "twist", twist).linear
    sample.update({"dvl_vx": float(v.x), "dvl_vy": float(v.y), "dvl_vz": float(v.z)})


def add_odom(sample: dict[str, Any], local_odom: Any) -> None:
    if local_odom is None:
        return
    p = local_odom.pose.pose.position
    v = local_odom.twist.twist.linear
    sample.update(
        {
            "odom_x": float(p.x),
            "odom_y": float(p.y),
            "odom_z": float(p.z),
            "odom_vx": float(v.x),
            "odom_vy": float(v.y),
            "odom_vz": float(v.z),
        }
    )


def add_depth(sample: dict[str, Any], depth: Any) -> None:
    if depth is not None:
        sample["depth_m"] = float(depth.data)


__all__ = ["add_depth", "add_dvl", "add_imu", "add_odom"]
