#!/usr/bin/env python3
"""Runtime contract checker for strict MuJoCo/real-package parity."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict

import rclpy
from dvl_msgs.msg import DVL
from hit25_auv_ros2_msg.msg import CollectorState
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


EXPECTED_TYPES = {
    "/mavros/state": "mavros_msgs/msg/State",
    "/dvl/data": "dvl_msgs/msg/DVL",
    "/dvl/twist": "geometry_msgs/msg/TwistWithCovarianceStamped",
    "/depth/pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/odometry/filtered": "nav_msgs/msg/Odometry",
    "/camera/camera/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "/collector/state": "hit25_auv_ros2_msg/msg/CollectorState",
}
SIMULATOR_NODE = "uuv_mujoco_bridge"


class ParityProbe(Node):
    def __init__(self) -> None:
        super().__init__("real_stack_parity_checker")
        self.counts: dict[str, int] = defaultdict(int)
        self.first_wall: dict[str, float] = {}
        self.last_wall: dict[str, float] = {}
        self.latest: dict[str, object] = {}
        self.create_subscription(State, "/mavros/state", lambda msg: self._record("/mavros/state", msg), 10)
        self.create_subscription(DVL, "/dvl/data", lambda msg: self._record("/dvl/data", msg), 10)
        self.create_subscription(Odometry, "/odometry/filtered", lambda msg: self._record("/odometry/filtered", msg), 10)
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            "/camera/camera/color/image_raw/compressed",
            lambda msg: self._record("/camera/camera/color/image_raw/compressed", msg),
            10,
        )
        self.create_subscription(RCOut, "/mavros/rc/out", lambda msg: self._record("/mavros/rc/out", msg), 10)
        self.create_subscription(
            CollectorState, "/collector/state", lambda msg: self._record("/collector/state", msg), 10
        )
        self.create_subscription(
            String,
            "/uuv_mujoco/sitl/mavlink_telemetry_status",
            lambda msg: self._record("/uuv_mujoco/sitl/mavlink_telemetry_status", msg),
            10,
        )

    def _record(self, topic: str, msg: object) -> None:
        now = time.monotonic()
        self.counts[topic] += 1
        self.first_wall.setdefault(topic, now)
        self.last_wall[topic] = now
        self.latest[topic] = msg

    def rate(self, topic: str) -> float:
        count = self.counts.get(topic, 0)
        elapsed = self.last_wall.get(topic, 0.0) - self.first_wall.get(topic, 0.0)
        return float(count - 1) / elapsed if count > 1 and elapsed > 0.0 else 0.0


class Results:
    def __init__(self) -> None:
        self.failures = 0

    def pass_(self, text: str) -> None:
        print(f"[PASS] {text}")

    def fail(self, text: str) -> None:
        self.failures += 1
        print(f"[FAIL] {text}")

    def warn(self, text: str) -> None:
        print(f"[WARN] {text}")

    def not_run(self, text: str) -> None:
        print(f"[NOT RUN] {text}")


def _node_name(info: object) -> str:
    namespace = str(getattr(info, "node_namespace", "")).rstrip("/")
    name = str(getattr(info, "node_name", ""))
    return f"{namespace}/{name}" if namespace else f"/{name}"


def _publishers(node: Node, topic: str) -> list[object]:
    return list(node.get_publishers_info_by_topic(topic))


def _subscriptions(node: Node, topic: str) -> list[object]:
    return list(node.get_subscriptions_info_by_topic(topic))


def _types(node: Node) -> dict[str, list[str]]:
    return {name: types for name, types in node.get_topic_names_and_types()}


def _check_types(node: Node, results: Results) -> None:
    actual = _types(node)
    for topic, expected in EXPECTED_TYPES.items():
        topic_types = actual.get(topic, [])
        if topic_types == [expected]:
            results.pass_(f"{topic} type={expected}")
        elif not topic_types:
            results.fail(f"{topic} missing; expected type={expected}")
        else:
            results.fail(f"{topic} type mismatch actual={topic_types} expected={expected}")


def _check_ownership(node: Node, results: Results, require_controller: bool) -> None:
    for topic, expected_count in (("/dvl/twist", 1), ("/depth/pose", 1), ("/odometry/filtered", 1)):
        publishers = _publishers(node, topic)
        names = [_node_name(info) for info in publishers]
        if len(publishers) == expected_count and not any(SIMULATOR_NODE in name for name in names):
            results.pass_(f"{topic} publisher_count=1 owner={names[0]}")
        else:
            results.fail(f"{topic} publishers={names}; expected one non-simulator owner")

    rc_publishers = _publishers(node, "/mavros/rc/override")
    rc_names = [_node_name(info) for info in rc_publishers if "real_stack_parity_checker" not in _node_name(info)]
    if len(rc_names) == 1:
        results.pass_(f"/mavros/rc/override publisher_count=1 owner={rc_names[0]}")
    elif not rc_names and not require_controller:
        results.warn("/mavros/rc/override has no active controller; safe while mission/joy is idle")
    else:
        results.fail(f"/mavros/rc/override publishers={rc_names}; expected exactly one")

    tf_publishers = _publishers(node, "/tf")
    tf_names = [_node_name(info) for info in tf_publishers]
    sim_tf = [name for name in tf_names if SIMULATOR_NODE in name]
    if sim_tf:
        results.fail(f"strict mode simulator publishes dynamic TF: {sim_tf}")
    else:
        results.pass_(f"dynamic TF has no simulator authority; publishers={tf_names}")

    direct_subscribers = [
        _node_name(info)
        for info in _subscriptions(node, "/uuv_mujoco/sitl/command_override")
        if SIMULATOR_NODE in _node_name(info)
    ]
    if direct_subscribers:
        results.fail(f"strict direct command path is active: {direct_subscribers}")
    else:
        results.pass_("strict direct MuJoCo command subscription is absent")

    for topic, _topic_types in node.get_topic_names_and_types():
        if not topic.startswith("/mavros/"):
            continue
        simulator_publishers = [
            _node_name(info) for info in _publishers(node, topic) if SIMULATOR_NODE in _node_name(info)
        ]
        if simulator_publishers:
            results.fail(f"simulator-owned MAVROS publisher topic={topic} nodes={simulator_publishers}")

    ground_truth_topics = [
        topic for topic, _ in node.get_topic_names_and_types()
        if topic.startswith("/mujoco/ground_truth") or topic == "/mujoco/course_buoys/status"
    ]
    offenders = []
    for topic in ground_truth_topics:
        for info in _subscriptions(node, topic):
            name = _node_name(info)
            if "mission" in name or "fsm" in name or "controller" in name:
                offenders.append(f"{name}:{topic}")
    if offenders:
        results.fail(f"mission/controller ground-truth subscriptions={offenders}")
    else:
        results.pass_("mission/controller has no ground-truth subscription")


def _check_samples(probe: ParityProbe, results: Results) -> None:
    state = probe.latest.get("/mavros/state")
    if state is not None and bool(getattr(state, "connected", False)):
        results.pass_("/mavros/state connected=true and fresh")
    else:
        results.fail("/mavros/state did not report connected=true")

    dvl = probe.latest.get("/dvl/data")
    if dvl is None:
        results.fail("/dvl/data has no fresh sample")
    elif not bool(getattr(dvl, "velocity_valid", False)):
        results.fail("/dvl/data velocity_valid=false")
    elif len(getattr(dvl, "covariance", [])) != 9 or getattr(dvl.header, "frame_id", "") != "dvl":
        results.fail("/dvl/data schema values invalid: require frame_id=dvl and covariance[9]")
    else:
        results.pass_("/dvl/data is fresh, velocity_valid=true, frame=dvl, covariance[9]")

    odom_rate = probe.rate("/odometry/filtered")
    if odom_rate >= 10.0:
        results.pass_(f"/odometry/filtered fresh rate={odom_rate:.1f}Hz")
    else:
        results.fail(f"/odometry/filtered rate={odom_rate:.1f}Hz; require >=10Hz")

    image = probe.latest.get("/camera/camera/color/image_raw/compressed")
    image_rate = probe.rate("/camera/camera/color/image_raw/compressed")
    if image is not None and len(getattr(image, "data", b"")) > 0 and image_rate > 0.5:
        results.pass_(f"compressed camera fresh rate={image_rate:.1f}Hz bytes={len(image.data)}")
    else:
        results.fail("compressed camera topic is missing, empty, or stale")

    collector = probe.latest.get("/collector/state")
    if collector is not None and getattr(collector.header, "frame_id", "") == "base_link":
        results.pass_(f"/collector/state fresh state={collector.state} frame=base_link")
    else:
        results.fail("/collector/state is missing, stale, or has a non-base_link frame")

    status_msg = probe.latest.get("/uuv_mujoco/sitl/mavlink_telemetry_status")
    try:
        status = json.loads(status_msg.data) if status_msg is not None else {}
    except (AttributeError, json.JSONDecodeError):
        status = {}
    if status.get("json_servo_frame_count") is not None and float(status.get("json_servo_age_s", 999.0)) < 1.0:
        results.pass_(
            f"UDP 9002 servo active frame={status.get('json_servo_frame_count')} rate={status.get('json_servo_frame_rate_hz')}Hz"
        )
    else:
        results.fail(f"UDP 9002 servo activity missing/stale status={status}")
    if int(status.get("json_sensor_packets_sent", 0)) > 0:
        results.pass_(f"UDP 9003 sensor active packets_sent={status['json_sensor_packets_sent']}")
    else:
        results.fail("UDP 9003 sensor activity missing")


def _exercise_rc_path(probe: ParityProbe, results: Results) -> None:
    state = probe.latest.get("/mavros/state")
    if state is None or not bool(getattr(state, "armed", False)):
        results.fail("RC exercise requires an already armed simulator; checker will not arm a vehicle")
        return
    publisher = probe.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
    command = OverrideRCIn()
    command.channels = [1500, 1500, 1500, 1500, 1800, 1500] + [65535] * 12
    baseline = list(getattr(probe.latest.get("/mavros/rc/out"), "channels", []))
    deadline = time.monotonic() + 3.0
    changed = None
    while time.monotonic() < deadline:
        publisher.publish(command)
        rclpy.spin_once(probe, timeout_sec=0.02)
        channels = list(getattr(probe.latest.get("/mavros/rc/out"), "channels", []))
        if channels and channels != baseline and any(value != 1500 for value in channels[:8]):
            changed = channels
            break
    release = OverrideRCIn()
    release.channels = [0] * 18
    publisher.publish(release)
    probe.destroy_publisher(publisher)
    if changed is None:
        results.fail(f"RC override produced no non-neutral /mavros/rc/out change; baseline={baseline}")
    else:
        results.pass_(f"RC override -> ArduSub -> servo output changed baseline={baseline[:8]} output={changed[:8]}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--require-controller", action="store_true")
    parser.add_argument("--exercise-rc", action="store_true")
    args = parser.parse_args()
    results = Results()

    rclpy.init()
    probe = ParityProbe()
    deadline = time.monotonic() + max(args.timeout, 1.0)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if probe.camera_subscription is not None and probe.counts["/camera/camera/color/image_raw/compressed"] >= 2:
                probe.destroy_subscription(probe.camera_subscription)
                probe.camera_subscription = None
            if (
                probe.counts["/odometry/filtered"] >= 60
                and probe.counts["/dvl/data"] >= 2
                and probe.counts["/mavros/state"] >= 1
                and probe.counts["/camera/camera/color/image_raw/compressed"] >= 2
                and probe.counts["/collector/state"] >= 2
                and probe.counts["/uuv_mujoco/sitl/mavlink_telemetry_status"] >= 1
            ):
                break
        _check_types(probe, results)
        _check_ownership(probe, results, args.require_controller)
        _check_samples(probe, results)
        if args.exercise_rc:
            _exercise_rc_path(probe, results)
        else:
            results.not_run("RC command -> SERVO_OUTPUT_RAW change; rerun a controlled armed test with --exercise-rc")
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    outcome = "PASS" if results.failures == 0 else "FAIL"
    print(f"\nREAL_STACK_PARITY={outcome} failures={results.failures}")
    return 0 if results.failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
