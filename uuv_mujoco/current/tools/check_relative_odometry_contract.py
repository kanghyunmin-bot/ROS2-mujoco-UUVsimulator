#!/usr/bin/env python3
"""Compare filtered odometry with MuJoCo using only t0-relative SE(3) motion.

The tool is deliberately read-only.  It subscribes to ``/sim/odom`` as a test
oracle and to ``/odometry/filtered`` as the real-vehicle localization surface;
it never publishes commands and never compares or records absolute positions.

By default the first measured window must remain stationary.  After a short
transition window, move or rotate the vehicle during the moving window.  Each
window gets its own reference transform, so map/odom origins and initial yaw do
not affect the reported XY, z, or yaw errors.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from relative_odometry_contract import (
    ErrorThresholds,
    PhaseMotionThresholds,
    PoseSample,
    analyze_phase,
    match_samples_by_stamp,
    normalize_quaternion,
)


CURRENT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = CURRENT_DIR / "generated" / "relative_odometry_contract.json"


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def positive_float(value: str) -> float:
    parsed = nonnegative_float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def _add_error_threshold_args(
    parser: argparse.ArgumentParser,
    phase: str,
    *,
    xy_rms: float,
    xy_max: float,
    z_rms: float,
    z_max: float,
    yaw_rms: float,
    yaw_max: float,
) -> None:
    prefix = f"--{phase}-max"
    parser.add_argument(f"{prefix}-xy-rms-m", type=nonnegative_float, default=xy_rms)
    parser.add_argument(f"{prefix}-xy-error-m", type=nonnegative_float, default=xy_max)
    parser.add_argument(f"{prefix}-z-rms-m", type=nonnegative_float, default=z_rms)
    parser.add_argument(f"{prefix}-z-error-m", type=nonnegative_float, default=z_max)
    parser.add_argument(
        f"{prefix}-yaw-rms-deg", type=nonnegative_float, default=yaw_rms
    )
    parser.add_argument(
        f"{prefix}-yaw-error-deg", type=nonnegative_float, default=yaw_max
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oracle-topic", default="/sim/odom")
    parser.add_argument("--estimate-topic", default="/odometry/filtered")
    parser.add_argument("--oracle-frame", default="map")
    parser.add_argument("--estimate-frame", default="odom")
    parser.add_argument("--child-frame", default="base_link")
    parser.add_argument("--expected-oracle-publishers", type=positive_int, default=1)
    parser.add_argument("--expected-estimate-publishers", type=positive_int, default=1)
    parser.add_argument("--settle-s", type=nonnegative_float, default=2.0)
    parser.add_argument("--stationary-s", type=positive_float, default=8.0)
    parser.add_argument("--transition-s", type=nonnegative_float, default=2.0)
    parser.add_argument("--moving-s", type=positive_float, default=15.0)
    parser.add_argument("--max-pair-dt-s", type=nonnegative_float, default=0.050)
    parser.add_argument("--min-phase-pairs", type=positive_int, default=30)
    parser.add_argument("--max-samples-per-topic", type=positive_int, default=100000)
    parser.add_argument(
        "--stationary-oracle-translation-max-m",
        type=nonnegative_float,
        default=0.10,
    )
    parser.add_argument(
        "--stationary-oracle-yaw-max-deg", type=nonnegative_float, default=5.0
    )
    parser.add_argument(
        "--moving-oracle-translation-min-m", type=nonnegative_float, default=0.20
    )
    parser.add_argument(
        "--moving-oracle-yaw-min-deg", type=nonnegative_float, default=5.0
    )
    _add_error_threshold_args(
        parser,
        "stationary",
        xy_rms=0.05,
        xy_max=0.10,
        z_rms=0.03,
        z_max=0.06,
        yaw_rms=2.0,
        yaw_max=4.0,
    )
    _add_error_threshold_args(
        parser,
        "moving",
        xy_rms=0.30,
        xy_max=0.60,
        z_rms=0.20,
        z_max=0.40,
        yaw_rms=6.0,
        yaw_max=12.0,
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _enum_name(value: object) -> str:
    return str(getattr(value, "name", value))


def publisher_info_payload(info: object) -> dict[str, object]:
    qos = getattr(info, "qos_profile", None)
    endpoint_gid = getattr(info, "endpoint_gid", b"")
    try:
        gid = bytes(endpoint_gid).hex()
    except (TypeError, ValueError):
        gid = str(endpoint_gid)
    qos_payload: dict[str, object] | None = None
    if qos is not None:
        qos_payload = {
            "depth": int(getattr(qos, "depth", 0)),
            "reliability": _enum_name(getattr(qos, "reliability", "unknown")),
            "durability": _enum_name(getattr(qos, "durability", "unknown")),
            "history": _enum_name(getattr(qos, "history", "unknown")),
        }
    return {
        "node_name": str(getattr(info, "node_name", "")),
        "node_namespace": str(getattr(info, "node_namespace", "")),
        "topic_type": str(getattr(info, "topic_type", "")),
        "endpoint_gid": gid,
        "qos": qos_payload,
    }


class TopicCapture:
    def __init__(self, topic: str, max_samples: int) -> None:
        self.topic = topic
        self.max_samples = max_samples
        self.samples: list[PoseSample] = []
        self.invalid_messages = 0
        self.dropped_messages = 0
        self.stamp_regressions = 0
        self.frames: Counter[str] = Counter()
        self.child_frames: Counter[str] = Counter()
        self._last_stamp_s: float | None = None

    def record(self, message: object, elapsed_s: float) -> None:
        try:
            header = getattr(message, "header")
            stamp = getattr(header, "stamp")
            pose = getattr(getattr(message, "pose"), "pose")
            position = getattr(pose, "position")
            orientation = getattr(pose, "orientation")
            stamp_s = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
            position_xyz = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            quaternion = normalize_quaternion(
                (
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                )
            )
            frame_id = str(header.frame_id)
            child_frame_id = str(getattr(message, "child_frame_id"))
        except (AttributeError, TypeError, ValueError):
            self.invalid_messages += 1
            return
        if not math.isfinite(stamp_s) or not all(math.isfinite(v) for v in position_xyz):
            self.invalid_messages += 1
            return
        if self._last_stamp_s is not None and stamp_s + 1.0e-9 < self._last_stamp_s:
            self.stamp_regressions += 1
        self._last_stamp_s = stamp_s
        self.frames[frame_id] += 1
        self.child_frames[child_frame_id] += 1
        if len(self.samples) >= self.max_samples:
            self.dropped_messages += 1
            return
        self.samples.append(
            PoseSample(
                stamp_s=stamp_s,
                elapsed_s=float(elapsed_s),
                position=position_xyz,
                orientation_xyzw=quaternion,
                frame_id=frame_id,
                child_frame_id=child_frame_id,
            )
        )


def build_topic_contract(
    capture: TopicCapture,
    publishers: list[dict[str, object]],
    *,
    expected_frame: str,
    expected_child_frame: str,
    expected_publishers: int,
) -> dict[str, object]:
    observed_frames = dict(sorted(capture.frames.items()))
    observed_child_frames = dict(sorted(capture.child_frames.items()))
    checks = {
        "publisher_count": {
            "actual": len(publishers),
            "expected": expected_publishers,
            "ok": len(publishers) == expected_publishers,
        },
        "message_count": {
            "actual": len(capture.samples),
            "minimum": 1,
            "ok": bool(capture.samples),
        },
        "header_frame": {
            "observed": observed_frames,
            "expected_only": expected_frame,
            "ok": set(observed_frames) == {expected_frame},
        },
        "child_frame": {
            "observed": observed_child_frames,
            "expected_only": expected_child_frame,
            "ok": set(observed_child_frames) == {expected_child_frame},
        },
        "valid_messages": {
            "invalid": capture.invalid_messages,
            "dropped": capture.dropped_messages,
            "ok": capture.invalid_messages == 0 and capture.dropped_messages == 0,
        },
        "monotonic_header_stamps": {
            "regressions": capture.stamp_regressions,
            "ok": capture.stamp_regressions == 0,
        },
    }
    failures = [name for name, check in checks.items() if not bool(check["ok"])]
    return {
        "topic": capture.topic,
        "message_type": "nav_msgs/msg/Odometry",
        "publishers": publishers,
        "observed_message_count": len(capture.samples),
        "observed_stamp_range_s": (
            {
                "first": capture.samples[0].stamp_s,
                "last": capture.samples[-1].stamp_s,
            }
            if capture.samples
            else None
        ),
        "checks": checks,
        "failures": failures,
        "ok": not failures,
    }


def _thresholds_from_args(args: argparse.Namespace, phase: str) -> ErrorThresholds:
    return ErrorThresholds(
        xy_rms_m=getattr(args, f"{phase}_max_xy_rms_m"),
        xy_max_m=getattr(args, f"{phase}_max_xy_error_m"),
        z_rms_m=getattr(args, f"{phase}_max_z_rms_m"),
        z_max_m=getattr(args, f"{phase}_max_z_error_m"),
        yaw_rms_deg=getattr(args, f"{phase}_max_yaw_rms_deg"),
        yaw_max_deg=getattr(args, f"{phase}_max_yaw_error_deg"),
    )


def capture_runtime(args: argparse.Namespace) -> tuple[TopicCapture, TopicCapture, dict[str, list[dict[str, object]]]]:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    # Reassigned immediately after node construction so ROS initialization and
    # subscription setup never consume the requested settle/phase windows.
    capture_started = time.monotonic()
    oracle = TopicCapture(args.oracle_topic, args.max_samples_per_topic)
    estimate = TopicCapture(args.estimate_topic, args.max_samples_per_topic)
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    class RelativeOdometryCheckNode(Node):
        def __init__(self) -> None:
            super().__init__("relative_odometry_contract_check")
            self._subscriptions = [
                self.create_subscription(
                    Odometry,
                    args.oracle_topic,
                    lambda message: oracle.record(
                        message, time.monotonic() - capture_started
                    ),
                    qos,
                ),
                self.create_subscription(
                    Odometry,
                    args.estimate_topic,
                    lambda message: estimate.record(
                        message, time.monotonic() - capture_started
                    ),
                    qos,
                ),
            ]

    rclpy.init()
    node = RelativeOdometryCheckNode()
    capture_started = time.monotonic()
    total_s = args.settle_s + args.stationary_s + args.transition_s + args.moving_s
    stationary_start_s = args.settle_s
    moving_start_s = args.settle_s + args.stationary_s + args.transition_s
    print(
        f"[relative_odom] keep vehicle stationary from {stationary_start_s:.1f}s "
        f"to {stationary_start_s + args.stationary_s:.1f}s",
        file=sys.stderr,
        flush=True,
    )
    print(
        f"[relative_odom] move or rotate vehicle from {moving_start_s:.1f}s "
        f"to {total_s:.1f}s",
        file=sys.stderr,
        flush=True,
    )
    moving_announced = False
    try:
        while rclpy.ok():
            elapsed_s = time.monotonic() - capture_started
            if elapsed_s >= total_s:
                break
            if not moving_announced and elapsed_s >= moving_start_s:
                print(
                    "[relative_odom] MOVING window started",
                    file=sys.stderr,
                    flush=True,
                )
                moving_announced = True
            rclpy.spin_once(node, timeout_sec=min(0.05, total_s - elapsed_s))
        publisher_payloads = {
            "oracle": [
                publisher_info_payload(info)
                for info in node.get_publishers_info_by_topic(args.oracle_topic)
            ],
            "estimate": [
                publisher_info_payload(info)
                for info in node.get_publishers_info_by_topic(args.estimate_topic)
            ],
        }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return oracle, estimate, publisher_payloads


def build_report(
    args: argparse.Namespace,
    oracle: TopicCapture,
    estimate: TopicCapture,
    publishers: dict[str, list[dict[str, object]]],
) -> dict[str, object]:
    pairs = match_samples_by_stamp(
        oracle.samples, estimate.samples, args.max_pair_dt_s
    )
    stationary_start_s = args.settle_s
    stationary_end_s = stationary_start_s + args.stationary_s
    moving_start_s = stationary_end_s + args.transition_s
    moving_end_s = moving_start_s + args.moving_s
    motion_thresholds = PhaseMotionThresholds(
        stationary_translation_max_m=args.stationary_oracle_translation_max_m,
        stationary_yaw_max_deg=args.stationary_oracle_yaw_max_deg,
        moving_translation_min_m=args.moving_oracle_translation_min_m,
        moving_yaw_min_deg=args.moving_oracle_yaw_min_deg,
    )
    phases = {
        "stationary": analyze_phase(
            "stationary",
            "stationary",
            pairs,
            window_start_s=stationary_start_s,
            window_end_s=stationary_end_s,
            min_pairs=args.min_phase_pairs,
            thresholds=_thresholds_from_args(args, "stationary"),
            motion_thresholds=motion_thresholds,
        ),
        "moving": analyze_phase(
            "moving",
            "moving",
            pairs,
            window_start_s=moving_start_s,
            window_end_s=moving_end_s,
            min_pairs=args.min_phase_pairs,
            thresholds=_thresholds_from_args(args, "moving"),
            motion_thresholds=motion_thresholds,
        ),
    }
    topic_contracts = {
        "oracle": build_topic_contract(
            oracle,
            publishers.get("oracle", []),
            expected_frame=args.oracle_frame,
            expected_child_frame=args.child_frame,
            expected_publishers=args.expected_oracle_publishers,
        ),
        "estimate": build_topic_contract(
            estimate,
            publishers.get("estimate", []),
            expected_frame=args.estimate_frame,
            expected_child_frame=args.child_frame,
            expected_publishers=args.expected_estimate_publishers,
        ),
    }
    failures: list[str] = []
    for role, contract in topic_contracts.items():
        if not contract["ok"]:
            failures.append(f"{role} topic contract failed")
    for name, phase in phases.items():
        if not phase["ok"]:
            failures.append(f"{name} relative-motion checks failed")
    if not pairs:
        failures.append("no timestamp-synchronized oracle/estimate pairs")
    return {
        "schema_version": 1,
        "tool": "check_relative_odometry_contract.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "oracle_policy": {
            "topic": args.oracle_topic,
            "role": "test_oracle_only",
            "subscription_only": True,
            "publishes_commands": False,
            "controller_input_modified": False,
        },
        "comparison_contract": {
            "method": "per-phase T0-relative SE(3)",
            "formula": "delta_T = inverse(T_phase_start) * T_sample",
            "absolute_position_comparison_performed": False,
            "absolute_positions_recorded": False,
            "metrics": ["relative_xy_m", "relative_z_m", "relative_yaw_deg"],
        },
        "capture": {
            "timing_s": {
                "settle": args.settle_s,
                "stationary": args.stationary_s,
                "transition": args.transition_s,
                "moving": args.moving_s,
            },
            "pairing": {
                "max_stamp_delta_s": args.max_pair_dt_s,
                "matched_pair_count": len(pairs),
                "oracle_sample_count": len(oracle.samples),
                "estimate_sample_count": len(estimate.samples),
            },
            "phase_motion_thresholds": asdict(motion_thresholds),
        },
        "topic_contracts": topic_contracts,
        "phases": phases,
        "failures": failures,
        "result": "PASS" if not failures else "FAIL",
        "ok": not failures,
    }


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    oracle, estimate, publishers = capture_runtime(args)
    report = build_report(args, oracle, estimate, publishers)
    text = json.dumps(report, indent=2, sort_keys=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    print(f"[relative_odom] JSON: {args.output}", file=sys.stderr)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
