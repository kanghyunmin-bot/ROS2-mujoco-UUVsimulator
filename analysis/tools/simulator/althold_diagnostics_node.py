"""ROS2 node for recording ALT_HOLD closed-loop diagnostics."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from althold_diagnostics_callbacks import (
    _elapsed,
    _on_depth,
    _on_dvl_vel,
    _on_manual,
    _on_mavros_pose,
    _on_mavros_vel,
    _on_mujoco_pose,
    _on_pressure,
    _on_rc_in,
    _on_rc_out,
    _on_sim_odom,
    _on_sitl_mavlink_status,
    _on_sitl_sensor_status,
)
from althold_diagnostics_contract import Snapshot
from althold_diagnostics_ros_imports import load_ros_symbols
from althold_diagnostics_runtime import _finite, _finish, _sample
from althold_diagnostics_subscriptions import create_subscriptions


class AltHoldDiagnosticsLogger:
    _elapsed = _elapsed
    _finish = _finish
    _finite = staticmethod(_finite)
    _on_depth = _on_depth
    _on_dvl_vel = _on_dvl_vel
    _on_manual = _on_manual
    _on_mavros_pose = _on_mavros_pose
    _on_mavros_vel = _on_mavros_vel
    _on_mujoco_pose = _on_mujoco_pose
    _on_pressure = _on_pressure
    _on_rc_in = _on_rc_in
    _on_rc_out = _on_rc_out
    _on_sim_odom = _on_sim_odom
    _on_sitl_mavlink_status = _on_sitl_mavlink_status
    _on_sitl_sensor_status = _on_sitl_sensor_status
    _sample = _sample

    def __init__(self, args: argparse.Namespace) -> None:
        ros = load_ros_symbols()
        self.rclpy = ros.rclpy
        self.PoseStamped = ros.PoseStamped
        self.TwistStamped = ros.TwistStamped
        self.ManualControl = ros.ManualControl
        self.RCIn = ros.RCIn
        self.RCOut = ros.RCOut
        self.Odometry = ros.Odometry
        self.Float32 = ros.Float32
        self.String = ros.String
        self.sensor_qos = ros.qos_profile_sensor_data

        self.duration_s = float(args.duration)
        self.sample_hz = float(args.sample_hz)
        self.plot = bool(args.plot)
        self.output_dir = Path(args.output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.output_dir / f"althold_diag_{stamp}.csv"
        self.plot_path = self.output_dir / f"althold_diag_{stamp}.png"

        self.rclpy.init(args=None)
        self.node = ros.Node("althold_diagnostics_logger")
        self.state = Snapshot(t_s=0.0)
        self.rows: list[Snapshot] = []
        self.done = False
        self.start_wall = time.monotonic()
        self.last_mujoco_depth_t: float | None = None
        self.last_mujoco_depth_m: float | None = None
        self.last_sim_odom_t: float | None = None
        self.last_sim_odom_depth_m: float | None = None

        create_subscriptions(self)
