#!/usr/bin/env python3
"""Smoke checks for odometry publish builders."""

from __future__ import annotations

import pathlib
import sys
from types import SimpleNamespace

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_publish_builder_odometry import build_odometry_publish_builders  # noqa: E402


class AutoMsg:
    def __init__(self) -> None:
        self.covariance = [0.0] * 36

    def __getattr__(self, name: str):
        child = AutoMsg()
        setattr(self, name, child)
        return child


def _assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def _assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1.0e-9:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def main() -> int:
    bridge = SimpleNamespace(
        Odometry=AutoMsg,
        TFMessage=AutoMsg,
        TransformStamped=AutoMsg,
        _odom_pos=np.array([1.0, 2.0, 3.0], dtype=np.float64),
        _base_to_rovio=np.eye(3, dtype=np.float64),
    )
    state = SimpleNamespace(
        dvl_vel_body_ros=None,
        quat_ros=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rot_world_body=np.eye(3, dtype=np.float64),
        base_pos_enu=np.array([4.0, 6.0, 8.0], dtype=np.float64),
        base_vel_body_ros=np.array([0.1, 0.2, 0.3], dtype=np.float64),
        gyro_ros=np.array([0.4, 0.5, 0.6], dtype=np.float64),
    )

    builders = build_odometry_publish_builders(bridge, "stamp", state)
    _assert_equal(set(builders), {"odom_local", "rovio_odom", "sim_odom", "tf"}, "builder keys")

    odom_local = builders["odom_local"]()
    _assert_equal(odom_local, builders["odom_local"](), "odom_local lazy cache")
    _assert_equal(odom_local.header.frame_id, "odom", "odom local frame")
    _assert_equal(odom_local.child_frame_id, "base_link", "odom local child")
    _assert_close(odom_local.twist.twist.linear.x, 0.0, "odom local fallback velocity x")

    sim_odom = builders["sim_odom"]()
    _assert_equal(sim_odom.header.frame_id, "map", "sim odom frame")
    _assert_close(sim_odom.pose.pose.position.x, 4.0, "sim odom x")
    _assert_close(sim_odom.twist.twist.angular.z, 0.6, "sim odom angular z")

    tf_msg = builders["tf"]()
    _assert_equal(len(tf_msg.transforms), 2, "tf transform count")
    _assert_equal(tf_msg.transforms[0].header.frame_id, "map", "tf map parent")
    _assert_equal(tf_msg.transforms[0].child_frame_id, "odom", "tf odom child")
    _assert_close(tf_msg.transforms[0].transform.translation.x, 3.0, "map to odom x")

    print("odometry_publish_builders=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
