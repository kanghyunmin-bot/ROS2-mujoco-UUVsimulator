#!/usr/bin/env python3
"""Offline unit tests for the t0-relative odometry contract checker."""

from __future__ import annotations

import math
import unittest

import check_relative_odometry_contract as runtime
from relative_odometry_contract import (
    ErrorThresholds,
    MatchedPair,
    PhaseMotionThresholds,
    PoseSample,
    analyze_phase,
    match_samples_by_stamp,
    quaternion_multiply,
    rotate_vector,
)


def yaw_quaternion(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def compose_pose(
    parent_position: tuple[float, float, float],
    parent_orientation: tuple[float, float, float, float],
    child_position: tuple[float, float, float],
    child_orientation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    rotated = rotate_vector(parent_orientation, child_position)
    return (
        tuple(parent_position[index] + rotated[index] for index in range(3)),
        quaternion_multiply(parent_orientation, child_orientation),
    )


def sample(
    stamp_s: float,
    elapsed_s: float,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    *,
    frame: str,
) -> PoseSample:
    return PoseSample(
        stamp_s=stamp_s,
        elapsed_s=elapsed_s,
        position=position,
        orientation_xyzw=orientation,
        frame_id=frame,
        child_frame_id="base_link",
    )


def transformed_pair(
    index: int,
    elapsed_s: float,
    relative_position: tuple[float, float, float],
    relative_yaw_deg: float,
) -> MatchedPair:
    oracle_origin = ((100.0, -50.0, 7.0), yaw_quaternion(31.0))
    oracle_pose = compose_pose(
        oracle_origin[0],
        oracle_origin[1],
        relative_position,
        yaw_quaternion(relative_yaw_deg),
    )
    # A completely different world origin and heading.  Left-multiplying every
    # oracle pose by this fixed transform must disappear after T0 rebasing.
    world_alignment = ((-700.0, 250.0, -13.0), yaw_quaternion(-121.0))
    estimate_pose = compose_pose(
        world_alignment[0],
        world_alignment[1],
        oracle_pose[0],
        oracle_pose[1],
    )
    oracle_sample = sample(
        10.0 + index * 0.1,
        elapsed_s,
        oracle_pose[0],
        oracle_pose[1],
        frame="map",
    )
    estimate_sample = sample(
        10.002 + index * 0.1,
        elapsed_s + 0.001,
        estimate_pose[0],
        estimate_pose[1],
        frame="odom",
    )
    return MatchedPair(
        oracle=oracle_sample,
        estimate=estimate_sample,
        stamp_delta_s=0.002,
        elapsed_s=elapsed_s + 0.001,
    )


class RelativeOdometryMathTest(unittest.TestCase):
    def test_fixed_world_transform_is_removed_by_phase_t0(self) -> None:
        pairs = [
            transformed_pair(0, 1.0, (0.0, 0.0, 0.0), 0.0),
            transformed_pair(1, 2.0, (0.8, -0.2, 0.1), 12.0),
            transformed_pair(2, 3.0, (1.6, 0.3, -0.4), 35.0),
        ]
        result = analyze_phase(
            "moving",
            "moving",
            pairs,
            window_start_s=0.5,
            window_end_s=3.5,
            min_pairs=3,
            thresholds=ErrorThresholds(
                xy_rms_m=1.0e-9,
                xy_max_m=1.0e-9,
                z_rms_m=1.0e-9,
                z_max_m=1.0e-9,
                yaw_rms_deg=1.0e-9,
                yaw_max_deg=1.0e-9,
            ),
            motion_thresholds=PhaseMotionThresholds(),
        )
        self.assertTrue(result["ok"], result)
        errors = result["errors"]
        self.assertLess(errors["relative_xy_m"]["max_abs"], 1.0e-10)
        self.assertLess(errors["relative_z_m"]["max_abs"], 1.0e-10)
        self.assertLess(errors["relative_yaw_deg"]["max_abs"], 1.0e-10)

    def test_known_relative_errors_and_yaw_wrap_are_reported(self) -> None:
        identity = yaw_quaternion(0.0)
        oracle = [
            sample(0.0, 0.0, (0.0, 0.0, 0.0), identity, frame="map"),
            sample(1.0, 1.0, (1.0, 0.0, 0.0), yaw_quaternion(179.0), frame="map"),
        ]
        estimate = [
            sample(0.0, 0.0, (50.0, 70.0, 9.0), identity, frame="odom"),
            sample(
                1.0,
                1.0,
                (51.3, 70.0, 9.2),
                yaw_quaternion(-179.0),
                frame="odom",
            ),
        ]
        pairs = [
            MatchedPair(oracle[i], estimate[i], 0.0, float(i)) for i in range(2)
        ]
        result = analyze_phase(
            "moving",
            "moving",
            pairs,
            window_start_s=0.0,
            window_end_s=2.0,
            min_pairs=2,
            thresholds=ErrorThresholds(
                xy_max_m=0.25,
                z_max_m=0.15,
                yaw_max_deg=1.5,
            ),
            motion_thresholds=PhaseMotionThresholds(
                moving_translation_min_m=0.5, moving_yaw_min_deg=5.0
            ),
        )
        self.assertFalse(result["ok"])
        errors = result["errors"]
        self.assertAlmostEqual(errors["relative_xy_m"]["max_abs"], 0.3, places=9)
        self.assertAlmostEqual(errors["relative_z_m"]["max_abs"], 0.2, places=9)
        self.assertAlmostEqual(errors["relative_yaw_deg"]["max_abs"], 2.0, places=9)
        self.assertEqual(
            set(result["failures"]), {"xy_max_m", "z_max_m", "yaw_max_deg"}
        )

    def test_timestamp_pairing_is_unique_and_bounded(self) -> None:
        identity = yaw_quaternion(0.0)
        oracle = [
            sample(t, t, (t, 0.0, 0.0), identity, frame="map")
            for t in (1.00, 1.10, 1.20)
        ]
        estimate = [
            sample(t, t, (t, 0.0, 0.0), identity, frame="odom")
            for t in (1.02, 1.11, 1.35)
        ]
        pairs = match_samples_by_stamp(oracle, estimate, max_stamp_delta_s=0.03)
        self.assertEqual(len(pairs), 2)
        self.assertEqual(len({id(pair.estimate) for pair in pairs}), 2)
        self.assertTrue(all(abs(pair.stamp_delta_s) <= 0.03 for pair in pairs))


class RelativeOdometryReportTest(unittest.TestCase):
    def test_report_contains_contracts_but_no_absolute_pose_payload(self) -> None:
        args = runtime.parse_args(
            [
                "--settle-s",
                "0",
                "--stationary-s",
                "1",
                "--transition-s",
                "0",
                "--moving-s",
                "2",
                "--min-phase-pairs",
                "2",
            ]
        )
        oracle = runtime.TopicCapture("/sim/odom", 100)
        estimate = runtime.TopicCapture("/odometry/filtered", 100)
        pairs = [
            transformed_pair(0, 0.1, (0.0, 0.0, 0.0), 0.0),
            transformed_pair(1, 0.6, (0.0, 0.0, 0.0), 0.0),
            transformed_pair(2, 1.1, (0.0, 0.0, 0.0), 0.0),
            transformed_pair(3, 2.5, (1.0, 0.0, 0.0), 15.0),
        ]
        oracle.samples.extend(pair.oracle for pair in pairs)
        estimate.samples.extend(pair.estimate for pair in pairs)
        oracle.frames["map"] = len(pairs)
        estimate.frames["odom"] = len(pairs)
        oracle.child_frames["base_link"] = len(pairs)
        estimate.child_frames["base_link"] = len(pairs)
        publishers = {
            "oracle": [{"node_name": "sim_bridge"}],
            "estimate": [{"node_name": "ekf_filter_node"}],
        }
        report = runtime.build_report(args, oracle, estimate, publishers)
        self.assertTrue(report["ok"], report)
        self.assertEqual(report["oracle_policy"]["role"], "test_oracle_only")
        self.assertFalse(
            report["comparison_contract"]["absolute_position_comparison_performed"]
        )

        forbidden_keys = {"position", "absolute_position", "absolute_xyz", "pose"}

        def walk(value: object) -> None:
            if isinstance(value, dict):
                self.assertTrue(forbidden_keys.isdisjoint(value.keys()))
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)

        walk(report)


if __name__ == "__main__":
    unittest.main()
