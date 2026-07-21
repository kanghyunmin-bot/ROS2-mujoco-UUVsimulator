#!/usr/bin/env python3
"""Focused timing tests for check_pinger_homing_deadline.py."""

from __future__ import annotations

import json
import unittest
from unittest import mock

import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

import check_pinger_homing_deadline as deadline


def clock_message(seconds: float) -> Clock:
    message = Clock()
    message.clock.sec = int(seconds)
    message.clock.nanosec = int(round((seconds - int(seconds)) * 1.0e9))
    return message


def controller_message(state: str, active: bool = True) -> String:
    message = String()
    message.data = json.dumps(
        {
            "state": state,
            "control_output_active": active,
            "acoustic_estimator_mode": "phase",
            "controller_profile": "real",
            "direction_frame": "world",
        }
    )
    return message


def oracle_message(range_m: float) -> String:
    message = String()
    message.data = json.dumps(
        {
            "range_m": range_m,
            "direction_body": [1.0, 0.0, 0.0],
            "direction_world": [0.0, 1.0, 0.0],
        }
    )
    return message


class DeadlineOracleTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def test_prestart_sample_hold_and_success_are_latched(self) -> None:
        node = deadline.DeadlineOracle(success_range_m=1.0, success_hold_s=0.5)
        self.addCleanup(node.destroy_node)

        node._on_clock(clock_message(100.0))
        with mock.patch.object(deadline.time, "monotonic", return_value=10.0):
            node._on_oracle(oracle_message(0.8))
        with mock.patch.object(deadline.time, "monotonic", return_value=10.05):
            node._on_controller(controller_message("PROBE"))

        self.assertEqual(node.range_samples, 1)
        self.assertEqual(node.start_wall_s, 10.05)
        self.assertEqual(node.success_started_wall_s, 10.05)

        # Wall time alone is insufficient: the oracle must hold in simulation
        # time as well.
        node._on_clock(clock_message(100.2))
        with mock.patch.object(deadline.time, "monotonic", return_value=10.70):
            node._on_oracle(oracle_message(0.7))
        self.assertFalse(node.oracle_hold_verified)

        node._on_clock(clock_message(100.6))
        with mock.patch.object(deadline.time, "monotonic", return_value=10.75):
            node._on_oracle(oracle_message(0.6))
        self.assertTrue(node.oracle_hold_verified)
        self.assertGreaterEqual(node.oracle_hold_wall_s or 0.0, 0.5)
        self.assertGreaterEqual(node.oracle_hold_sim_s or 0.0, 0.5)
        self.assertGreaterEqual(node.oracle_hold_samples, 2)
        first_success = (node.success_wall_s, node.success_sim_s)

        # Continued in-range samples and a later state message must not move the
        # first success crossing or erase a COMPLETE observation.
        node._on_clock(clock_message(120.0))
        with mock.patch.object(deadline.time, "monotonic", return_value=30.0):
            node._on_oracle(oracle_message(0.5))
        self.assertEqual((node.success_wall_s, node.success_sim_s), first_success)
        with mock.patch.object(deadline.time, "monotonic", return_value=30.1):
            node._on_controller(controller_message("COMPLETE"))
        with mock.patch.object(deadline.time, "monotonic", return_value=30.2):
            node._on_controller(controller_message("WAIT_VEHICLE", active=False))
        self.assertTrue(node.controller_complete)
        self.assertEqual((node.success_wall_s, node.success_sim_s), first_success)

    def test_first_post_start_clock_anchors_sim_stopwatch(self) -> None:
        node = deadline.DeadlineOracle(success_range_m=1.0, success_hold_s=0.2)
        self.addCleanup(node.destroy_node)

        with mock.patch.object(deadline.time, "monotonic", return_value=5.0):
            node._on_controller(controller_message("PROBE"))
        self.assertIsNone(node.start_sim_s)

        node._on_clock(clock_message(200.0))
        self.assertEqual(node.start_sim_s, 200.0)
        with mock.patch.object(deadline.time, "monotonic", return_value=5.1):
            node._on_oracle(oracle_message(0.8))
        node._on_clock(clock_message(200.25))
        with mock.patch.object(deadline.time, "monotonic", return_value=5.35):
            node._on_oracle(oracle_message(0.7))
        self.assertTrue(node.oracle_hold_verified)
        self.assertAlmostEqual(node.success_sim_s or -1.0, 0.25, places=6)


if __name__ == "__main__":
    unittest.main()
