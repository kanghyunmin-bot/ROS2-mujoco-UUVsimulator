"""Text-formatting helpers for MuJoCo viewer overlays."""

from __future__ import annotations


def command_overlay_text(*, forward: float, sway: float, yaw: float, heave: float, camera_mode: str) -> str:
    return (
        f"Cmd fwd {forward:+.1f} sway {sway:+.1f} "
        f"yaw {yaw:+.1f} heave {heave:+.1f} | Cam: {camera_mode}"
    )


def sensor_overlay_text(*, imu_g, imu_a, dvl_v, dvl_alt, depth_pos) -> str:
    if imu_g is None or imu_a is None or dvl_v is None or dvl_alt is None:
        return ""
    depth_suffix = ""
    if depth_pos is not None and len(depth_pos) >= 3:
        depth_suffix = f" depth_z={depth_pos[2]:+.2f}"
    return (
        f"IMU gyro[{imu_g[0]:+.2f},{imu_g[1]:+.2f},{imu_g[2]:+.2f}] "
        f"acc[{imu_a[0]:+.2f},{imu_a[1]:+.2f},{imu_a[2]:+.2f}] "
        f"DVL vel[{dvl_v[0]:+.2f},{dvl_v[1]:+.2f},{dvl_v[2]:+.2f}] alt={dvl_alt[0]:.2f}m"
        f"{depth_suffix}"
    )


__all__ = ["command_overlay_text", "sensor_overlay_text"]
