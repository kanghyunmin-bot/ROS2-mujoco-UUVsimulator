#!/usr/bin/env python3
"""Record SLAM estimate and evaluation-only MuJoCo truth as TUM files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from sim.evaluation.trajectory_recording import PoseRecordBuffer  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--estimate-topic", default="/odometry/filtered")
    parser.add_argument("--ground-truth-topic", default="/mujoco/ground_truth/pose")
    parser.add_argument(
        "--estimate-type",
        choices=("odometry", "pose_stamped"),
        default="odometry",
    )
    parser.add_argument(
        "--estimate-frame-id",
        default="odom",
        help="Required estimate header.frame_id.",
    )
    parser.add_argument(
        "--estimate-child-frame-id",
        default=None,
        help="Required Odometry child_frame_id; defaults to base_link.",
    )
    parser.add_argument(
        "--ground-truth-frame-id",
        default="world",
        help="Required evaluation-only ground-truth header.frame_id.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Wall duration; zero records until Ctrl-C.",
    )
    return parser.parse_args()


def _timestamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _append_pose(
    buffer: PoseRecordBuffer,
    header,
    pose,
    *,
    child_frame_id: str | None = None,
) -> None:
    buffer.append(
        _timestamp_s(header.stamp),
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        frame_id=header.frame_id,
        child_frame_id=child_frame_id,
    )


def main() -> int:
    args = parse_args()
    if args.duration_s < 0.0:
        raise ValueError("--duration-s must be non-negative")

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    estimate_child_frame_id = args.estimate_child_frame_id
    if args.estimate_type == "odometry" and estimate_child_frame_id is None:
        estimate_child_frame_id = "base_link"
    if args.estimate_type == "pose_stamped" and estimate_child_frame_id is not None:
        raise ValueError("--estimate-child-frame-id is valid only for odometry")

    estimate = PoseRecordBuffer(
        "estimate",
        expected_frame_id=args.estimate_frame_id,
        expected_child_frame_id=estimate_child_frame_id,
    )
    ground_truth = PoseRecordBuffer(
        "ground_truth_evaluation_only",
        expected_frame_id=args.ground_truth_frame_id,
    )
    rclpy.init()
    node = rclpy.create_node("slam_trajectory_evaluation_recorder")
    estimate_type = Odometry if args.estimate_type == "odometry" else PoseStamped

    def on_estimate(message) -> None:
        pose = message.pose.pose if args.estimate_type == "odometry" else message.pose
        child_frame_id = (
            message.child_frame_id if args.estimate_type == "odometry" else None
        )
        _append_pose(
            estimate,
            message.header,
            pose,
            child_frame_id=child_frame_id,
        )

    def on_ground_truth(message: PoseStamped) -> None:
        _append_pose(ground_truth, message.header, message.pose)

    estimate_subscription = node.create_subscription(
        estimate_type,
        args.estimate_topic,
        on_estimate,
        qos_profile_sensor_data,
    )
    truth_subscription = node.create_subscription(
        PoseStamped,
        args.ground_truth_topic,
        on_ground_truth,
        qos_profile_sensor_data,
    )
    del estimate_subscription, truth_subscription

    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration_s > 0.0 and time.monotonic() - started >= args.duration_s:
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    estimate.write_tum(args.output_dir / "estimate.tum")
    ground_truth.write_tum(args.output_dir / "ground_truth.tum")
    metadata = {
        "schema": "uuv_mujoco.slam_trajectory_recording.v1",
        "estimate_topic": args.estimate_topic,
        "estimate_type": args.estimate_type,
        "ground_truth_topic": args.ground_truth_topic,
        "ground_truth_policy": "evaluation_only",
        "duration_wall_s": time.monotonic() - started,
        "interrupted": interrupted,
        "estimate": estimate.summary(),
        "ground_truth": ground_truth.summary(),
    }
    (args.output_dir / "recording_metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))
    if estimate.rejected_frame_contract or ground_truth.rejected_frame_contract:
        return 5
    if len(estimate.records) < 2 or len(ground_truth.records) < 2:
        return 2
    if estimate.rejected_nonmonotonic or ground_truth.rejected_nonmonotonic:
        return 3
    if estimate.rejected_invalid or ground_truth.rejected_invalid:
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
