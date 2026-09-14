#!/usr/bin/env python3
"""Observe VLA inputs without publishing controls or starting an episode."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from itertools import pairwise
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from auv_dvl_a50_msg.msg import DVL
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import State
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Imu

ROOT = Path(__file__).resolve().parents[3]
COLLECTOR = ROOT / "rospkg/src/auv_vla_data_collector"
sys.path.insert(0, str(COLLECTOR))
from kmu26_auv_vla_data_collector.contract import (
    body_velocity,
    normalize_quaternion_wxyz,
    sample_is_fresh,
)


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {
        "min": float(values.min()),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


class InputProbe(Node):
    """Subscribe to the collector surface; retain metadata, never image payloads."""

    def __init__(self, config):
        super().__init__(
            "vla_live_input_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.config = config
        self.latest = {}
        self.streams = {}
        self.rows = []
        self.measuring = False
        self.vehicle = None
        self.vehicle_wall = None
        self.vehicle_ros = None
        self.vehicle_events = []
        self.clock_first = self.clock_last = None
        self.clock_changes = self.clock_resets = 0
        self.clock_progress_wall = time.monotonic()
        self.clock_max_stall = 0.0
        self.camera_decode_wall = {}
        self.camera_info = {}
        subscriptions = [
            ("ego", "ego_image_topic", CompressedImage),
            ("hand", "buoy_release_image_topic", CompressedImage),
            ("imu", "imu_topic", Imu),
            ("depth", "depth_topic", PoseWithCovarianceStamped),
            ("dvl_twist", "dvl_twist_topic", TwistWithCovarianceStamped),
            ("dvl_data", "dvl_data_topic", DVL),
        ]
        if config.get("imu_motion_topic"):
            subscriptions.append(("imu_motion", "imu_motion_topic", Imu))
        for key, config_key, message_type in subscriptions:
            self.streams[key] = {"topic": config[config_key], "events": []}
            self.create_subscription(
                message_type,
                config[config_key],
                lambda message, name=key: self.receive(name, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            State, "/mavros/state", self.receive_vehicle, qos_profile_sensor_data
        )
        self.create_subscription(
            Clock, "/clock", self.receive_clock, qos_profile_sensor_data
        )
        self.create_timer(1.0 / float(config["record_rate_hz"]), self.sample)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive_vehicle(self, message):
        self.vehicle = {
            "connected": message.connected,
            "armed": message.armed,
            "mode": message.mode,
        }
        self.vehicle_wall = time.monotonic()
        self.vehicle_ros = self.now()
        if self.measuring:
            self.vehicle_events.append((self.now(), self.vehicle_wall))

    def receive_clock(self, message):
        current = (
            float(message.clock.sec) + message.clock.nanosec * 1e-9,
            time.monotonic(),
        )
        if not self.measuring:
            return
        if self.clock_first is None:
            self.clock_first = current
            self.clock_progress_wall = current[1]
        if self.clock_last is not None and current[0] != self.clock_last[0]:
            self.clock_resets += int(current[0] < self.clock_last[0])
            self.clock_changes += 1
            self.clock_max_stall = max(
                self.clock_max_stall, current[1] - self.clock_progress_wall
            )
            self.clock_progress_wall = current[1]
        self.clock_last = current

    def receive(self, key, message):
        wall, received = time.monotonic(), self.now()
        source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        frame = message.header.frame_id
        valid, extra = True, {}
        if key in ("ego", "hand"):
            # Decode at most once per wall second per camera to keep this probe cheap.
            if wall - self.camera_decode_wall.get(key, -math.inf) >= 1.0:
                try:
                    image = cv2.imdecode(
                        np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR
                    )
                except cv2.error:
                    image = None
                self.camera_info[key] = (
                    None
                    if image is None
                    else {
                        "shape_hwc": list(image.shape),
                        "dtype": str(image.dtype),
                    }
                )
                self.camera_decode_wall[key] = wall
            extra = {
                "decoded_image": self.camera_info.get(key),
                "compressed_bytes": len(message.data),
            }
            valid = extra["decoded_image"] is not None
        elif key == "imu_motion":
            angular, linear = message.angular_velocity, message.linear_acceleration
            valid = (
                frame == self.config["imu_motion_frame"]
                and message.angular_velocity_covariance[0] >= 0
                and message.linear_acceleration_covariance[0] >= 0
                and bool(np.isfinite([angular.x, angular.y, angular.z, linear.x, linear.y, linear.z]).all())
            )
        elif key == "imu":
            q, angular, linear = (
                message.orientation,
                message.angular_velocity,
                message.linear_acceleration,
            )
            values = [angular.x, angular.y, angular.z, linear.x, linear.y, linear.z]
            valid = (
                frame == self.config["body_frame"]
                and message.orientation_covariance[0] >= 0
            )
            try:
                normalize_quaternion_wxyz([q.w, q.x, q.y, q.z])
                valid = valid and bool(np.isfinite(values).all())
            except ValueError:
                valid = False
        elif key == "depth":
            depth = float(message.pose.pose.position.z)
            if self.config["depth_pose_z_is_positive_up"]:
                depth = -depth
            valid = math.isfinite(depth)
            extra["depth_m"] = depth if valid else None
        elif key == "dvl_twist":
            velocity = message.twist.twist.linear
            try:
                body_velocity(
                    [velocity.x, velocity.y, velocity.z],
                    frame,
                    self.config["dvl_input_frame"],
                    self.config["dvl_convention"],
                )
            except ValueError:
                valid = False
        elif key == "dvl_data":
            extra = {
                "velocity_valid": bool(message.velocity_valid),
                "altitude_valid": bool(
                    message.velocity_valid
                    and math.isfinite(message.altitude)
                    and message.altitude > 0
                ),
            }
        event = {
            "source_time": source,
            "receipt_time": received,
            "wall_time": wall,
            "frame_id": frame,
            "semantic_valid": valid,
            **extra,
        }
        self.latest[key] = event
        if self.measuring:
            self.streams[key]["events"].append(event)

    def sample(self):
        if not self.measuring:
            return
        now = self.now()
        row = {"ros_time": now, "wall_time": time.monotonic(), "inputs": {}}
        for key in self.streams:
            value = self.latest.get(key)
            row["inputs"][key] = (
                None
                if value is None
                else {
                    "source_time": value["source_time"],
                    "source_age_s": now - value["source_time"],
                    "fresh": sample_is_fresh(
                        value["source_time"],
                        value["receipt_time"],
                        now,
                        float(self.config["max_sensor_age_sec"]),
                    ),
                    "semantic_valid": value["semantic_valid"],
                }
            )
        ego, hand = row["inputs"]["ego"], row["inputs"]["hand"]
        row["camera_skew_s"] = (
            None
            if ego is None or hand is None
            else abs(ego["source_time"] - hand["source_time"])
        )
        raw, twist = self.latest.get("dvl_data"), self.latest.get("dvl_twist")
        row["dvl_velocity_usable"] = bool(
            raw
            and twist
            and raw["velocity_valid"]
            and twist["semantic_valid"]
            and row["inputs"]["dvl_data"]["fresh"]
            and row["inputs"]["dvl_twist"]["fresh"]
            and abs(raw["source_time"] - twist["source_time"]) <= 1e-6
        )
        row["altitude_usable"] = bool(
            raw and raw["altitude_valid"] and row["inputs"]["dvl_data"]["fresh"]
        )
        self.rows.append(row)

    def report(self, wall_start, wall_end):
        elapsed = wall_end - wall_start
        clock_span = (
            0.0
            if self.clock_first is None
            else self.clock_last[0] - self.clock_first[0]
        )
        clock_wall_span = (
            0.0
            if self.clock_first is None
            else self.clock_last[1] - self.clock_first[1]
        )
        streams, failures = {}, []
        for key, stream in self.streams.items():
            events = stream["events"]
            samples = [row["inputs"][key] for row in self.rows]
            present = [value for value in samples if value is not None]
            sampled_stamps = [value["source_time"] for value in present]
            source_stamps = sorted({event["source_time"] for event in events})
            source_span = (
                source_stamps[-1] - source_stamps[0] if len(source_stamps) > 1 else 0.0
            )
            freshness = sum(
                value["fresh"] and value["semantic_valid"] for value in present
            ) / max(1, len(samples))
            repeated = sum(a >= b for a, b in pairwise(sampled_stamps))
            streams[key] = {
                "topic": stream["topic"],
                "publisher_count": self.count_publishers(stream["topic"]),
                "received_messages": len(events),
                "received_hz_wall_window": len(events) / elapsed,
                "unique_source_stamps": len(source_stamps),
                "observed_unique_source_hz": (len(source_stamps) - 1) / source_span
                if source_span > 0
                else None,
                "frame_ids": sorted({event["frame_id"] for event in events}),
                "collector_samples": len(samples),
                "samples_with_input": len(present),
                "missing_samples": len(samples) - len(present),
                "stale_samples": sum(not value["fresh"] for value in present),
                "semantic_invalid_samples": sum(
                    not value["semantic_valid"] for value in present
                ),
                "fresh_and_semantic_valid_ratio": freshness,
                "repeated_or_rewound_sample_stamps": repeated,
                "source_age_at_sample_s": distribution(
                    [value["source_age_s"] for value in present]
                ),
                "source_age_at_receipt_s": distribution(
                    [e["receipt_time"] - e["source_time"] for e in events]
                ),
                "invalid_sample_examples": [
                    {"sample_ros_time": row["ros_time"], "input": row["inputs"][key]}
                    for row in self.rows
                    if row["inputs"][key] is None
                    or not row["inputs"][key]["fresh"]
                    or not row["inputs"][key]["semantic_valid"]
                ][:20],
            }
            if key in ("ego", "hand"):
                streams[key]["decoded_image_checks"] = sorted(
                    {json.dumps(e["decoded_image"], sort_keys=True) for e in events}
                )
                streams[key]["decoded_image_checks"] = [
                    json.loads(v) for v in streams[key]["decoded_image_checks"]
                ]
            if not events or (
                key in ("ego", "hand", "imu", "imu_motion", "depth") and freshness < 1.0
            ):
                failures.append(f"{key}: missing, stale, or invalid inputs")
            if key in ("ego", "hand") and repeated:
                failures.append(
                    f"{key}: repeated or rewound frames at collector cadence"
                )
        max_stall = max(self.clock_max_stall, wall_end - self.clock_progress_wall)
        if not self.clock_changes or self.clock_resets or max_stall > 1.0:
            failures.append("simulation clock missing, reset, or stalled")
        vehicle_age = (
            None if self.vehicle_wall is None else wall_end - self.vehicle_wall
        )
        vehicle_ros_age = (
            None if self.vehicle_ros is None else self.now() - self.vehicle_ros
        )
        rc_publishers = self.count_publishers(self.config["rc_override_topic"])
        if (
            not self.vehicle
            or not self.vehicle["connected"]
            or not 0.0 <= vehicle_ros_age <= 2.0
        ):
            failures.append("fresh connected vehicle state unavailable")
        if not self.vehicle or not self.vehicle["armed"]:
            failures.append("vehicle disarmed; task demonstration not ready")
        if not self.vehicle or self.vehicle["mode"] != self.config["expected_mode"]:
            failures.append(f"vehicle mode differs from {self.config['expected_mode']}")
        if rc_publishers != 1:
            failures.append("RC override publisher count must be exactly one")
        return {
            "kind": "read_only_live_input_probe",
            "wall_duration_s": elapsed,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "collector_rate_hz_configured": self.config["record_rate_hz"],
            "sensor_max_age_s": self.config["max_sensor_age_sec"],
            "clock": {
                "sim_progress_s": clock_span,
                "observed_rtf": clock_span / clock_wall_span
                if clock_wall_span > 0
                else None,
                "progress_events": self.clock_changes,
                "resets": self.clock_resets,
                "max_no_progress_wall_s": max_stall,
            },
            "streams": streams,
            "collector_timer_samples": len(self.rows),
            "collector_sample_interval_s": distribution(
                np.diff([row["ros_time"] for row in self.rows])
            ),
            "camera_source_skew_s": distribution(
                [
                    row["camera_skew_s"]
                    for row in self.rows
                    if row["camera_skew_s"] is not None
                ]
            ),
            "dvl_velocity_usable_ratio": sum(
                row["dvl_velocity_usable"] for row in self.rows
            )
            / max(1, len(self.rows)),
            "altitude_usable_ratio": sum(row["altitude_usable"] for row in self.rows)
            / max(1, len(self.rows)),
            "vehicle": self.vehicle,
            "vehicle_receipt_age_wall_s": vehicle_age,
            "vehicle_receipt_age_ros_s": vehicle_ros_age,
            "vehicle_freshness_clock": "simulation",
            "vehicle_message_count": len(self.vehicle_events),
            "vehicle_receipt_interval_wall_s": distribution(
                np.diff([event[1] for event in self.vehicle_events])
            ),
            "vehicle_receipt_interval_sim_s": distribution(
                np.diff([event[0] for event in self.vehicle_events])
            ),
            "rc_override_publisher_count": rc_publishers,
            "blocking_observations": failures,
            "limits": "Observed rates are measured, not configured publication rates. Camera decoding is sampled once per wall second. No controls or collector services were called. This is not a task demonstration or physical calibration validation.",
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="Measurement wall seconds, 0.5..300",
    )
    parser.add_argument(
        "--warmup", type=float, default=2.0, help="DDS discovery wall seconds, 0..30"
    )
    parser.add_argument(
        "--collector_config", type=Path, default=COLLECTOR / "config/collector.yaml"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--imu_motion_topic", help="Observe independently timestamped simulation raw motion")
    args = parser.parse_args()
    if not 0.5 <= args.duration <= 300 or not 0 <= args.warmup <= 30:
        parser.error("duration must be 0.5..300 seconds and warmup 0..30 seconds")
    config = yaml.safe_load(args.collector_config.read_text())["vla_data_collector"][
        "ros__parameters"
    ]
    if args.imu_motion_topic is not None:
        config["imu_motion_topic"] = args.imu_motion_topic
    config.setdefault("imu_motion_frame", "fcu_link")
    for key in ("record_rate_hz", "max_sensor_age_sec"):
        if not math.isfinite(float(config[key])) or float(config[key]) <= 0:
            parser.error(f"{key} must be finite and positive")
    rclpy.init()
    node, executor = InputProbe(config), SingleThreadedExecutor()
    executor.add_node(node)
    try:
        warmup_end = time.monotonic() + args.warmup
        while time.monotonic() < warmup_end:
            executor.spin_once(
                timeout_sec=min(0.05, max(0, warmup_end - time.monotonic()))
            )
        start = time.monotonic()
        node.measuring = True
        node.clock_progress_wall = start
        while time.monotonic() - start < args.duration:
            executor.spin_once(
                timeout_sec=min(
                    0.05, max(0, args.duration - (time.monotonic() - start))
                )
            )
        result = node.report(start, time.monotonic())
        result["collector_config"] = str(args.collector_config.resolve())
        text = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text)
        print(text, end="")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
