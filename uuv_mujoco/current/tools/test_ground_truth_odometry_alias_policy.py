#!/usr/bin/env python3
"""Regression tests for exact-state ROS topic isolation."""

from __future__ import annotations

import argparse
import ast
from contextlib import redirect_stdout
import io
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.ros2_endpoint_core_publishers import create_core_sensor_publishers  # noqa: E402
from bridge.ros2_publish_schedule_hydrophone import schedule_hydrophone_jobs  # noqa: E402
from bridge.ros2_publish_schedule_odometry import schedule_odometry_ros_jobs  # noqa: E402
from bridge.ros2_topic_specs import (  # noqa: E402
    PUBLISHER_SPECS,
    UNSAFE_LEGACY_PUBLISHER_SPECS,
)
from bridge.ros2_topic_summaries import (  # noqa: E402
    build_bridge_topic_summary,
    build_ros2_bridge_active_log,
)
from sim.contracts.ground_truth import (  # noqa: E402
    UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI,
    UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV,
    UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING,
    resolve_unsafe_legacy_ground_truth_odometry_filtered,
)
from sim.runtime.cli_ros2 import add_ros2_args  # noqa: E402
from sim.runtime.ros_bridge_launch_status import (  # noqa: E402
    handle_ros_bridge_launch_error,
    print_ros_bridge_launch_summary,
)


class FakeNode:
    def __init__(self) -> None:
        self.publishers: list[str] = []

    def create_publisher(self, _msg_type, topic: str, _qos):
        self.publishers.append(topic)
        return topic


class FakeJobs:
    def __init__(self) -> None:
        self.entries: list[tuple[object, str, object, dict[str, object]]] = []

    def add(self, publisher, topic: str, builder, **kwargs) -> None:
        self.entries.append((publisher, topic, builder, kwargs))


def core_bridge(*, strict: bool, unsafe: bool) -> SimpleNamespace:
    bridge = SimpleNamespace(
        node=FakeNode(),
        _real_pkg_compat=strict,
        _unsafe_legacy_ground_truth_odometry_filtered=unsafe,
        CollectorState=None,
    )
    for attr in (
        "Clock",
        "Imu",
        "Float32",
        "PoseWithCovarianceStamped",
        "TwistStamped",
        "TwistWithCovarianceStamped",
        "Range",
        "Odometry",
        "BatteryState",
        "PoseStamped",
        "String",
    ):
        setattr(bridge, attr, object)
    return bridge


def schedule_bridge(*, strict: bool, unsafe: bool) -> SimpleNamespace:
    return SimpleNamespace(
        _real_pkg_compat=strict,
        _unsafe_legacy_ground_truth_odometry_filtered=unsafe,
        _mavros_surface_enabled=False,
        _ros_rate_dvl_position_hz=5.0,
        _ros_rate_mavros_local_position_hz=5.0,
        pub_dvl_odometry="dvl_odom_pub",
        pub_mavros_local_odom="mavros_local_odom_pub",
        pub_rovio_odometry="rovio_odom_pub",
        pub_sim_odometry="sim_odom_pub",
        pub_odometry_filtered="unsafe_filtered_pub" if unsafe else None,
    )


class GroundTruthOdometryAliasPolicyTest(unittest.TestCase):
    def test_constructor_keeps_unsafe_opt_in_after_existing_positionals(self) -> None:
        tree = ast.parse((CURRENT / "bridge" / "ros2_bridge.py").read_text())
        ros2_bridge = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "Ros2Bridge"
        )
        constructor = next(
            node
            for node in ros2_bridge.body
            if isinstance(node, ast.FunctionDef) and node.name == "__init__"
        )
        positional_names = [argument.arg for argument in constructor.args.args]
        self.assertEqual(
            positional_names[-1],
            "unsafe_legacy_ground_truth_odometry_filtered",
        )

    def test_cli_and_environment_are_default_off_and_explicit(self) -> None:
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop(UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV, None)
            parser = argparse.ArgumentParser()
            add_ros2_args(parser)
            default_args = parser.parse_args([])
        self.assertFalse(default_args.unsafe_legacy_ground_truth_odometry_filtered)

        with patch.dict(
            os.environ,
            {UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV: "1"},
            clear=False,
        ):
            parser = argparse.ArgumentParser()
            add_ros2_args(parser)
            environment_args = parser.parse_args([])
        self.assertTrue(environment_args.unsafe_legacy_ground_truth_odometry_filtered)

        with patch.dict(
            os.environ,
            {UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_ENV: "1"},
            clear=False,
        ):
            parser = argparse.ArgumentParser()
            add_ros2_args(parser)
            disabled_args = parser.parse_args(
                [
                    "--no-"
                    + UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI.removeprefix(
                        "--"
                    )
                ]
            )
        self.assertFalse(disabled_args.unsafe_legacy_ground_truth_odometry_filtered)

        parser = argparse.ArgumentParser()
        add_ros2_args(parser)
        cli_args = parser.parse_args(
            [UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_CLI]
        )
        self.assertTrue(cli_args.unsafe_legacy_ground_truth_odometry_filtered)

    def test_strict_mode_rejects_unsafe_alias(self) -> None:
        self.assertFalse(
            resolve_unsafe_legacy_ground_truth_odometry_filtered(
                requested=False,
                real_pkg_compat=False,
            )
        )
        self.assertTrue(
            resolve_unsafe_legacy_ground_truth_odometry_filtered(
                requested=True,
                real_pkg_compat=False,
            )
        )
        with self.assertRaisesRegex(ValueError, "external estimator"):
            resolve_unsafe_legacy_ground_truth_odometry_filtered(
                requested=True,
                real_pkg_compat=True,
            )

        strict_args = SimpleNamespace(
            ros2=True,
            sitl=False,
            ros2_real_pkg_compat=True,
            unsafe_legacy_ground_truth_odometry_filtered=True,
        )
        with self.assertRaisesRegex(
            SystemExit,
            "strict real-package compatibility rejected",
        ):
            handle_ros_bridge_launch_error(
                strict_args,
                ValueError("external estimator owns /odometry/filtered"),
            )

    def test_endpoint_is_absent_by_default_and_in_strict_mode(self) -> None:
        default_bridge = core_bridge(strict=False, unsafe=False)
        create_core_sensor_publishers(default_bridge, q10=10)
        self.assertNotIn("/odometry/filtered", default_bridge.node.publishers)
        self.assertIsNone(default_bridge.pub_odometry_filtered)

        unsafe_bridge = core_bridge(strict=False, unsafe=True)
        create_core_sensor_publishers(unsafe_bridge, q10=10)
        self.assertEqual(
            unsafe_bridge.node.publishers.count("/odometry/filtered"),
            1,
        )

        strict_bridge = core_bridge(strict=True, unsafe=True)
        create_core_sensor_publishers(strict_bridge, q10=10)
        self.assertNotIn("/odometry/filtered", strict_bridge.node.publishers)
        self.assertIsNone(strict_bridge.pub_odometry_filtered)

    def test_normal_odometry_jobs_survive_without_truth_alias(self) -> None:
        builders = {
            "odom_local": "local_builder",
            "mavros_local_odom": "mavros_builder",
            "rovio_odom": "rovio_builder",
            "sim_odom": "sim_builder",
        }

        def run(unsafe: bool):
            jobs = FakeJobs()
            rate_jobs: list[tuple[object, str, object, float]] = []
            schedule_odometry_ros_jobs(
                schedule_bridge(strict=False, unsafe=unsafe),
                jobs,
                lambda publisher, topic, builder, rate_hz: rate_jobs.append(
                    (publisher, topic, builder, rate_hz)
                ),
                builders=builders,
            )
            return jobs, rate_jobs

        default_jobs, default_rate_jobs = run(False)
        default_topics = {entry[1] for entry in default_jobs.entries}
        default_rate_topics = {entry[1] for entry in default_rate_jobs}
        self.assertEqual(default_topics, {"/rovio/odometry", "/sim/odom"})
        self.assertEqual(default_rate_topics, {"/dvl/odometry"})
        self.assertNotIn("/odometry/filtered", default_rate_topics)

        _, unsafe_rate_jobs = run(True)
        self.assertEqual(
            [entry[1] for entry in unsafe_rate_jobs].count("/odometry/filtered"),
            1,
        )

    def test_hydrophone_sync_cannot_reintroduce_default_alias(self) -> None:
        builders = {
            "depth_pose": "depth_builder",
            "sim_odom": "sim_builder",
            "hydrophone_status": "status_builder",
            "hydrophone_direction": "direction_builder",
            "hydrophone_audio_info": "info_builder",
            "hydrophone_audio": "audio_builder",
        }

        def run(unsafe: bool) -> list[str]:
            bridge = SimpleNamespace(
                _real_pkg_compat=False,
                _unsafe_legacy_ground_truth_odometry_filtered=unsafe,
                _hydrophone_config=SimpleNamespace(
                    enabled=True,
                    publish_hz=10.0,
                    info_hz=1.0,
                    status_hz=2.0,
                ),
                pub_depth_pose="depth_pub",
                pub_odometry_filtered="unsafe_filtered_pub" if unsafe else None,
                pub_hydrophone_status="status_pub",
                pub_hydrophone_direction="direction_pub",
                pub_hydrophone_audio_info="info_pub",
                pub_hydrophone_audio="audio_pub",
            )
            topics: list[str] = []
            schedule_hydrophone_jobs(
                bridge,
                FakeJobs(),
                lambda _publisher, topic, _builder, _rate_hz, **_kwargs: topics.append(
                    topic
                ),
                builders=builders,
            )
            return topics

        default_topics = run(False)
        self.assertIn("/depth/pose:hydrophone_sync", default_topics)
        self.assertNotIn("/odometry/filtered:hydrophone_sync", default_topics)
        self.assertIn("/odometry/filtered:hydrophone_sync", run(True))

    def test_topic_summary_marks_oracle_and_unsafe_opt_in(self) -> None:
        default_summary = build_bridge_topic_summary(
            enable_ping360=False,
            real_pkg_compat=False,
        )
        self.assertNotIn("/odometry/filtered", default_summary)
        self.assertIn("simulation-only ground-truth oracle", default_summary)
        self.assertIn("forbidden for estimator/control", default_summary)

        unsafe_summary = build_bridge_topic_summary(
            enable_ping360=False,
            real_pkg_compat=False,
            unsafe_legacy_ground_truth_odometry_filtered=True,
        )
        self.assertIn("/odometry/filtered(UNSAFE LEGACY", unsafe_summary)

        default_active = build_ros2_bridge_active_log(mavros_surface_enabled=True)
        unsafe_active = build_ros2_bridge_active_log(
            mavros_surface_enabled=True,
            unsafe_legacy_ground_truth_odometry_filtered=True,
        )
        self.assertNotIn("/odometry/filtered", default_active)
        self.assertIn("UNSAFE LEGACY", unsafe_active)

        args = SimpleNamespace(
            no_ping360=True,
            ros2_real_pkg_compat=False,
            ros2=True,
            sitl=False,
            unsafe_legacy_ground_truth_odometry_filtered=True,
            ros2_images=False,
            sitl_mavlink_endpoint="",
        )
        output = io.StringIO()
        with redirect_stdout(output):
            print_ros_bridge_launch_summary(args, enable_ros2=True)
        self.assertIn(
            UNSAFE_LEGACY_GROUND_TRUTH_ODOMETRY_FILTERED_WARNING,
            output.getvalue(),
        )

    def test_topic_registry_partitions_unsafe_alias_from_defaults(self) -> None:
        default_topics = {spec.topic for spec in PUBLISHER_SPECS}
        unsafe_specs = {
            spec.name: spec.topic for spec in UNSAFE_LEGACY_PUBLISHER_SPECS
        }
        self.assertNotIn("/odometry/filtered", default_topics)
        self.assertEqual(
            unsafe_specs,
            {"odometry_filtered": "/odometry/filtered"},
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
