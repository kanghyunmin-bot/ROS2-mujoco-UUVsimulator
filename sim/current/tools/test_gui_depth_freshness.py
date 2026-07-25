#!/usr/bin/env python3
"""Regression checks for independent pressure and depth freshness."""

from __future__ import annotations

import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from gui.models import TelemetrySnapshot  # noqa: E402
from gui.node_motion_depth_callbacks import _on_pressure_value  # noqa: E402
from gui.node_motion_pose_callbacks import _on_ground_truth_pose, _on_odom  # noqa: E402
from gui.node_snapshot_runtime import snapshot  # noqa: E402
from gui.readiness_feedback_gate import depth_feedback_reason  # noqa: E402


class _Owner:
    def __init__(self) -> None:
        self._snapshot = TelemetrySnapshot()
        self._last_wall: dict[str, float] = {}
        self._lock = threading.Lock()

    def _touch(self, key: str) -> None:
        self._last_wall[key] = time.monotonic()


def _pose(z: float) -> SimpleNamespace:
    position = SimpleNamespace(x=0.0, y=0.0, z=z)
    return SimpleNamespace(pose=SimpleNamespace(position=position))


def _odom(x: float, y: float, z: float) -> SimpleNamespace:
    position = SimpleNamespace(x=x, y=y, z=z)
    linear = SimpleNamespace(x=0.1, y=0.2, z=0.3)
    return SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(position=position)),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=linear)),
    )


class GuiDepthFreshnessTest(unittest.TestCase):
    def test_filtered_odometry_updates_depth_without_dedicated_depth_topic(self) -> None:
        owner = _Owner()

        _on_odom(owner, _odom(1.0, 2.0, -0.8), "/odometry/filtered")
        self.assertAlmostEqual(owner._snapshot.depth_m, 0.8, places=6)
        _on_odom(owner, _odom(1.2, 2.1, -1.4), "/odometry/filtered")

        self.assertEqual(owner._snapshot.position_xyz, (1.2, 2.1, -1.4))
        self.assertAlmostEqual(owner._snapshot.depth_m, 1.4, places=6)
        self.assertEqual(owner._snapshot.depth_source, "/odometry/filtered.pose.z")

    def test_filtered_odometry_temporarily_wins_over_fallback_odometry(self) -> None:
        owner = _Owner()

        _on_odom(owner, _odom(2.0, 0.0, -1.0), "/odometry/filtered")
        _on_odom(owner, _odom(9.0, 0.0, -9.0), "/dvl/odometry")

        self.assertEqual(owner._snapshot.position_xyz, (2.0, 0.0, -1.0))
        self.assertEqual(owner._snapshot.velocity_source, "/odometry/filtered")

    def test_pose_derived_depth_uses_pose_freshness_for_arm_gate(self) -> None:
        owner = _Owner()
        owner._snapshot.depth_m = 0.6536
        owner._snapshot.depth_source = "/sim/odom.pose.z"
        owner._last_wall["pose"] = time.monotonic() - 0.05

        snap = snapshot(owner)

        self.assertLess(snap.depth_age_s, 0.5)
        self.assertEqual(depth_feedback_reason(snap), "")

    def test_direct_depth_does_not_borrow_unrelated_pose_freshness(self) -> None:
        owner = _Owner()
        owner._snapshot.depth_m = 0.7
        owner._snapshot.depth_source = "/depth"
        owner._last_wall["depth"] = time.monotonic() - 4.0
        owner._last_wall["pose"] = time.monotonic() - 0.05

        snap = snapshot(owner)

        self.assertGreaterEqual(snap.depth_age_s, 3.0)
        self.assertEqual(depth_feedback_reason(snap), "waiting for fresh Bar30/depth feedback")

    def test_pressure_does_not_keep_old_pose_depth_fresh(self) -> None:
        owner = _Owner()
        owner._snapshot.depth_m = 0.6536
        owner._snapshot.depth_source = "/sim/odom.pose.z"
        owner._last_wall["depth"] = time.monotonic() - 2.0

        old_depth_stamp = owner._last_wall["depth"]
        _on_pressure_value(owner, 163000.0, "/mavros/imu/atm_pressure")

        self.assertEqual(owner._last_wall["depth"], old_depth_stamp)
        self.assertIn("pressure", owner._last_wall)
        _on_ground_truth_pose(owner, _pose(-6.09))
        self.assertAlmostEqual(owner._snapshot.depth_m, 6.09, places=6)
        self.assertEqual(owner._snapshot.depth_source, "/mujoco/ground_truth/pose.z")

    def test_pressure_only_depth_remains_live_and_updates(self) -> None:
        owner = _Owner()
        self.assertTrue(math.isnan(owner._snapshot.depth_m))
        _on_pressure_value(owner, 111101.65, "/bar30/pressure_pa")
        first_depth = owner._snapshot.depth_m
        first_stamp = owner._last_wall["depth"]
        _on_pressure_value(owner, 120878.30, "/bar30/pressure_pa")
        self.assertGreater(owner._snapshot.depth_m, first_depth)
        self.assertGreaterEqual(owner._last_wall["depth"], first_stamp)
        self.assertEqual(owner._snapshot.depth_source, "/bar30/pressure_pa (approx)")


if __name__ == "__main__":
    unittest.main()
