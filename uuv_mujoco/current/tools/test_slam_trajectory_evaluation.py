#!/usr/bin/env python3
"""Regression tests for evaluation-only SLAM trajectory metrics."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path
import sys

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from sim.evaluation.trajectory_metrics import (  # noqa: E402
    Trajectory,
    evaluate_trajectory,
    load_tum_trajectory,
)
from sim.evaluation.trajectory_recording import PoseRecordBuffer  # noqa: E402


def yaw_quaternion(yaw_rad: np.ndarray | float) -> np.ndarray:
    yaw = np.asarray(yaw_rad, dtype=np.float64)
    zeros = np.zeros_like(yaw)
    return np.stack(
        (zeros, zeros, np.sin(0.5 * yaw), np.cos(0.5 * yaw)),
        axis=-1,
    )


def reference_trajectory() -> Trajectory:
    times = np.linspace(0.0, 2.0, 21)
    positions = np.column_stack(
        (
            0.4 * times,
            0.2 * np.sin(1.7 * times),
            -0.1 * times + 0.03 * np.cos(times),
        )
    )
    return Trajectory(times, positions, yaw_quaternion(0.25 * times))


class TrajectoryValidationTest(unittest.TestCase):
    def test_rejects_duplicate_time_and_zero_quaternion(self) -> None:
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            Trajectory(
                np.array((0.0, 0.0)),
                np.zeros((2, 3)),
                np.tile((0.0, 0.0, 0.0, 1.0), (2, 1)),
            )
        with self.assertRaisesRegex(ValueError, "zero-norm"):
            Trajectory(np.array((0.0, 1.0)), np.zeros((2, 3)), np.zeros((2, 4)))

    def test_tum_loader_preserves_order_and_rejects_bad_rows(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            valid = Path(directory) / "valid.tum"
            valid.write_text(
                "# t tx ty tz qx qy qz qw\n"
                "0.0 0 0 0 0 0 0 1\n"
                "0.1,1,2,3,0,0,0,1\n",
                encoding="utf-8",
            )
            trajectory = load_tum_trajectory(valid)
            np.testing.assert_allclose(trajectory.positions_m[-1], (1.0, 2.0, 3.0))

            invalid = Path(directory) / "invalid.tum"
            invalid.write_text("0 0 0\n1 0 0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "expected 8 columns"):
                load_tum_trajectory(invalid)

    def test_record_buffer_rejects_timestamp_faults_without_hiding_them(self) -> None:
        buffer = PoseRecordBuffer("estimate")
        self.assertTrue(buffer.append(0.0, (0, 0, 0), (0, 0, 0, 2)))
        self.assertTrue(buffer.append(0.1, (1, 0, 0), (0, 0, 0, 1)))
        self.assertFalse(buffer.append(0.1, (2, 0, 0), (0, 0, 0, 1)))
        self.assertFalse(buffer.append(0.2, (2, 0, 0), (0, 0, 0, 0)))
        self.assertEqual(buffer.rejected_nonmonotonic, 1)
        self.assertEqual(buffer.rejected_invalid, 1)
        self.assertEqual(buffer.records[0].quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.tum"
            buffer.write_tum(path)
            loaded = load_tum_trajectory(path)
            self.assertEqual(loaded.timestamps_s.tolist(), [0.0, 0.1])

    def test_record_buffer_enforces_frame_and_child_frame_contract(self) -> None:
        buffer = PoseRecordBuffer(
            "estimate",
            expected_frame_id="odom",
            expected_child_frame_id="base_link",
        )
        self.assertTrue(
            buffer.append(
                0.0,
                (0, 0, 0),
                (0, 0, 0, 1),
                frame_id="odom",
                child_frame_id="base_link",
            )
        )
        self.assertFalse(
            buffer.append(
                0.1,
                (1, 0, 0),
                (0, 0, 0, 1),
                frame_id="map",
                child_frame_id="base_link",
            )
        )
        self.assertFalse(
            buffer.append(
                0.2,
                (2, 0, 0),
                (0, 0, 0, 1),
                frame_id="odom",
                child_frame_id="base_footprint",
            )
        )
        summary = buffer.summary()
        self.assertEqual(summary["rejected_frame_contract_count"], 2)
        self.assertEqual(summary["observed_frame_ids"], ["map", "odom"])
        self.assertEqual(
            summary["observed_child_frame_ids"],
            ["base_footprint", "base_link"],
        )


class TrajectoryMetricTest(unittest.TestCase):
    def test_se3_alignment_removes_only_global_rigid_transform(self) -> None:
        truth = reference_trajectory()
        global_yaw = 0.7
        rotation = np.asarray(
            [
                [np.cos(global_yaw), -np.sin(global_yaw), 0.0],
                [np.sin(global_yaw), np.cos(global_yaw), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        translation = np.asarray((1.2, -0.8, 0.4))
        estimate_positions = (rotation.T @ (truth.positions_m - translation).T).T
        estimate = Trajectory(
            truth.timestamps_s,
            estimate_positions,
            yaw_quaternion(0.25 * truth.timestamps_s - global_yaw),
        )

        metrics = evaluate_trajectory(
            estimate,
            truth,
            alignment="se3",
            rpe_delta_s=0.5,
            rpe_tolerance_s=1.0e-6,
        )

        self.assertLess(metrics["ate_translation_m"]["rmse"], 1.0e-12)
        self.assertLess(metrics["ate_rotation_deg"]["rmse"], 1.0e-6)
        self.assertLess(metrics["rpe_translation_m"]["rmse"], 1.0e-12)
        self.assertEqual(metrics["rpe_pair_count"], 16)
        self.assertAlmostEqual(metrics["alignment_scale"], 1.0)

    def test_sim3_alignment_recovers_metric_scale(self) -> None:
        truth = reference_trajectory()
        scale = 2.5
        estimate = Trajectory(
            truth.timestamps_s,
            truth.positions_m / scale,
            truth.quaternions_xyzw,
        )

        metrics = evaluate_trajectory(estimate, truth, alignment="sim3")

        self.assertAlmostEqual(metrics["alignment_scale"], scale, places=12)
        self.assertLess(metrics["ate_translation_m"]["rmse"], 1.0e-12)

    def test_explicit_clock_offset_synchronizes_measurements(self) -> None:
        truth = reference_trajectory()
        estimate = Trajectory(
            truth.timestamps_s + 0.03,
            truth.positions_m,
            truth.quaternions_xyzw,
        )

        metrics = evaluate_trajectory(
            estimate,
            truth,
            alignment="none",
            estimate_time_offset_s=-0.03,
            rpe_delta_s=0.5,
            rpe_tolerance_s=1.0e-6,
        )

        self.assertEqual(metrics["matched_sample_count"], 21)
        self.assertEqual(metrics["match_ratio"], 1.0)
        self.assertLess(metrics["ate_translation_m"]["rmse"], 1.0e-12)

    def test_interpolation_and_gap_gate_are_explicit(self) -> None:
        truth = reference_trajectory()
        query_times = truth.timestamps_s[:-1] + 0.05
        positions = np.column_stack(
            (
                0.4 * query_times,
                0.2 * np.sin(1.7 * query_times),
                -0.1 * query_times + 0.03 * np.cos(query_times),
            )
        )
        estimate = Trajectory(query_times, positions, yaw_quaternion(0.25 * query_times))

        metrics = evaluate_trajectory(
            estimate,
            truth,
            alignment="none",
            max_interpolation_gap_s=0.100001,
        )
        self.assertEqual(metrics["matched_sample_count"], 20)
        self.assertLess(metrics["ate_translation_m"]["rmse"], 6.0e-4)
        with self.assertRaisesRegex(ValueError, "fewer than three"):
            evaluate_trajectory(
                estimate,
                truth,
                alignment="none",
                max_interpolation_gap_s=0.05,
            )

    def test_drift_remains_visible_after_alignment(self) -> None:
        truth = reference_trajectory()
        drift = np.column_stack(
            (0.04 * truth.timestamps_s**2, np.zeros(21), np.zeros(21))
        )
        estimate = Trajectory(
            truth.timestamps_s,
            truth.positions_m + drift,
            truth.quaternions_xyzw,
        )

        metrics = evaluate_trajectory(
            estimate,
            truth,
            alignment="se3",
            rpe_delta_s=0.5,
            rpe_tolerance_s=1.0e-6,
        )

        self.assertGreater(metrics["ate_translation_m"]["rmse"], 0.01)
        self.assertGreater(metrics["rpe_translation_m"]["rmse"], 0.01)


class TrajectoryCliTest(unittest.TestCase):
    def test_cli_writes_json_and_markdown_artifacts(self) -> None:
        rows = "\n".join(
            f"{0.1 * index} {index} 0 0 0 0 0 1" for index in range(5)
        )
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            estimate = directory_path / "estimate.tum"
            truth = directory_path / "truth.tum"
            json_out = directory_path / "metrics.json"
            markdown_out = directory_path / "metrics.md"
            estimate.write_text(rows + "\n", encoding="utf-8")
            truth.write_text(rows + "\n", encoding="utf-8")

            completed = subprocess.run(
                [
                    sys.executable,
                    str(CURRENT / "tools" / "evaluate_slam_trajectory.py"),
                    "--estimate",
                    str(estimate),
                    "--ground-truth",
                    str(truth),
                    "--rpe-delta-s",
                    "0.2",
                    "--rpe-tolerance-s",
                    "0.000001",
                    "--json-out",
                    str(json_out),
                    "--markdown-out",
                    str(markdown_out),
                ],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            payload = json.loads(json_out.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], "uuv_mujoco.slam_trajectory_metrics.v1")
            self.assertEqual(payload["matched_sample_count"], 5)
            self.assertIn("Ground truth is evaluation-only", markdown_out.read_text())


if __name__ == "__main__":
    unittest.main(verbosity=2)
