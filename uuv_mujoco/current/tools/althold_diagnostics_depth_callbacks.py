"""Depth and vertical-velocity callbacks for ALT_HOLD diagnostics."""

from __future__ import annotations

import math


def quat_to_euler_rad(q) -> tuple[float, float, float]:
    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def elapsed(owner) -> float:
    return owner._elapsed()


def on_depth(owner, msg) -> None:
    owner.state.depth_bar30_m = float(getattr(msg, "data", math.nan))


def on_pressure(owner, msg) -> None:
    owner.state.pressure_bar30_pa = float(getattr(msg, "data", math.nan))


def on_mavros_pose(owner, msg) -> None:
    owner.state.mavros_local_depth_m = max(0.0, -float(msg.pose.position.z))
    roll, pitch, yaw = quat_to_euler_rad(msg.pose.orientation)
    owner.state.mavros_roll_rad = roll
    owner.state.mavros_pitch_rad = pitch
    owner.state.mavros_yaw_rad = yaw


def on_mavros_vel(owner, msg) -> None:
    owner.state.mavros_velz_down_mps = -float(msg.twist.linear.z)


def on_dvl_vel(owner, msg) -> None:
    owner.state.dvl_velz_down_mps = -float(msg.twist.linear.z)


def on_mujoco_pose(owner, msg) -> None:
    now = elapsed(owner)
    depth_m = max(0.0, -float(msg.pose.position.z))
    owner.state.mujoco_depth_m = depth_m
    roll, pitch, yaw = quat_to_euler_rad(msg.pose.orientation)
    owner.state.mujoco_roll_rad = roll
    owner.state.mujoco_pitch_rad = pitch
    owner.state.mujoco_yaw_rad = yaw
    if owner.last_mujoco_depth_t is not None and owner.last_mujoco_depth_m is not None:
        dt = now - owner.last_mujoco_depth_t
        if 1.0e-4 <= dt <= 1.0:
            owner.state.mujoco_depth_rate_down_mps = (depth_m - owner.last_mujoco_depth_m) / dt
    owner.last_mujoco_depth_t = now
    owner.last_mujoco_depth_m = depth_m


def on_sim_odom(owner, msg) -> None:
    now = elapsed(owner)
    depth_m = max(0.0, -float(msg.pose.pose.position.z))
    owner.state.sim_odom_depth_m = depth_m
    roll, pitch, yaw = quat_to_euler_rad(msg.pose.pose.orientation)
    owner.state.sim_odom_roll_rad = roll
    owner.state.sim_odom_pitch_rad = pitch
    owner.state.sim_odom_yaw_rad = yaw
    if owner.last_sim_odom_t is not None and owner.last_sim_odom_depth_m is not None:
        dt = now - owner.last_sim_odom_t
        if 1.0e-4 <= dt <= 1.0:
            owner.state.sim_odom_depth_rate_down_mps = (depth_m - owner.last_sim_odom_depth_m) / dt
    owner.last_sim_odom_t = now
    owner.last_sim_odom_depth_m = depth_m


__all__ = [
    "on_depth",
    "on_dvl_vel",
    "on_mavros_pose",
    "on_mavros_vel",
    "on_mujoco_pose",
    "on_pressure",
    "on_sim_odom",
    "quat_to_euler_rad",
]
