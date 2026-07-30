#!/usr/bin/env python3
"""Drive the hydrophone controller through confirm->neutral->grant with ROS inputs."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time

import rclpy
from audio_common_msgs.msg import Float64Stamped
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class HandoffProbe(Node):
    def __init__(self) -> None:
        super().__init__("hydrophone_vision_handoff_contract_probe")
        self.start_pub = self.create_publisher(
            PoseStamped, "/test/hydro/start_frame", latched_qos()
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/test/hydro/odom", 10
        )
        self.snr_pub = self.create_publisher(
            Float64Stamped, "/test/hydro/snr", 10
        )
        self.gradient_pub = self.create_publisher(
            Vector3Stamped, "/test/hydro/region_gradient", 10
        )
        self.confirm_pub = self.create_publisher(
            Bool, "/test/hydro/target_confirmed", latched_qos()
        )
        self.state = ""
        self.state_history: list[str] = []
        self.search_events: list[tuple[float, bool]] = []
        self.grant_events: list[tuple[float, bool]] = []
        self.rc_events: list[tuple[float, list[int]]] = []
        self.create_subscription(
            String, "/test/hydro/state", self._on_state, latched_qos()
        )
        self.create_subscription(
            Bool, "/test/hydro/vision_search", self._on_search, latched_qos()
        )
        self.create_subscription(
            Bool, "/test/hydro/vision_grant", self._on_grant, latched_qos()
        )
        self.create_subscription(
            OverrideRCIn, "/test/hydro/rc", self._on_rc, 10
        )

    def _on_state(self, msg: String) -> None:
        self.state = str(msg.data)
        if not self.state_history or self.state_history[-1] != self.state:
            self.state_history.append(self.state)

    def _on_search(self, msg: Bool) -> None:
        self.search_events.append((time.monotonic(), bool(msg.data)))

    def _on_grant(self, msg: Bool) -> None:
        self.grant_events.append((time.monotonic(), bool(msg.data)))

    def _on_rc(self, msg: OverrideRCIn) -> None:
        self.rc_events.append((time.monotonic(), list(msg.channels)))

    def publish_start(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.orientation.w = 1.0
        self.start_pub.publish(msg)

    def publish_odom(self, x: float, y: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def publish_gradient(self) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = 1.0
        self.gradient_pub.publish(msg)

    def publish_snr(self) -> None:
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = 80.0
        self.snr_pub.publish(msg)


def start_controller() -> subprocess.Popen[bytes]:
    parameters = {
        "odometry_topic": "/test/hydro/odom",
        "start_frame_topic": "/test/hydro/start_frame",
        "snr_topic": "/test/hydro/snr",
        "region_gradient_topic": "/test/hydro/region_gradient",
        "rolling_gradient_topic": "/test/hydro/rolling_gradient",
        "state_topic": "/test/hydro/state",
        "vision_search_request_topic": "/test/hydro/vision_search",
        "target_confirmed_topic": "/test/hydro/target_confirmed",
        "vision_control_granted_topic": "/test/hydro/vision_grant",
        "rc_override_topic": "/test/hydro/rc",
        "arena_length_m": "10.0",
        "arena_width_m": "10.0",
        "arena_safety_margin_m": "0.1",
        "use_explicit_initial_scan_center": "true",
        "initial_scan_center_x_m": "2.0",
        "initial_scan_center_y_m": "-2.0",
        "initial_scan_radius_m": "0.1",
        "rescan_radius_m": "0.1",
        "waypoint_reach_tolerance_m": "0.03",
        "scan_completion_radius_tolerance_m": "0.05",
        "vision_near_zone_width_m": "0.0",
        "vision_handoff_enabled": "true",
        "acoustic_timeout_s": "0.0",
        "success_snr_db": "70.0",
        "success_hold_s": "0.1",
        "success_snr_timeout_s": "1.0",
        "success_snr_window_size": "5",
        "target_depth_z_m": "0.0",
        "depth_tolerance_m": "0.05",
        "odometry_timeout_s": "1.0",
        "enable_keyboard_emergency_stop": "false",
        "rate_hz": "30.0",
    }
    command = [
        "ros2",
        "component",
        "standalone",
        "hydrophone_ctrl",
        "audio_capture::WaypointHomingControllerNode",
        "--node-name",
        "test_waypoint_homing_controller",
    ]
    for name, value in parameters.items():
        command.extend(["--parameter", f"{name}:={value}"])
    return subprocess.Popen(command, start_new_session=True)


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def spin_step(
    node: HandoffProbe,
    x: float,
    y: float,
    *,
    gradient: bool = False,
    snr: bool = False,
    duration_s: float = 0.08,
) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_start()
        node.publish_odom(x, y)
        if gradient:
            node.publish_gradient()
        if snr:
            node.publish_snr()
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_for_state(
    node: HandoffProbe, expected: str, x: float, y: float, timeout_s: float
) -> None:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline and node.state != expected:
        spin_step(node, x, y, gradient=True, duration_s=0.05)
    if node.state != expected:
        raise AssertionError(
            f"expected state={expected}, got={node.state}, "
            f"history={node.state_history}"
        )


def controlled_channels_are_neutral(channels: list[int]) -> bool:
    return all(channels[index] == 1500 for index in (2, 3, 4, 5))


def main() -> int:
    rclpy.init()
    node = HandoffProbe()
    process = start_controller()
    try:
        # Establish the guided frame and reach the explicit scan center/start.
        wait_for_state(node, "MOVE_TO_SCAN_START", 2.0, -2.0, 2.0)
        wait_for_state(node, "REGION_SCAN", 2.1, -2.0, 2.0)

        # One deterministic CCW lap; gradient is continuously available.
        for step in range(1, 46):
            angle = 2.0 * math.pi * step / 40.0
            x = 2.0 + 0.1 * math.cos(angle)
            y = -2.0 + 0.1 * math.sin(angle)
            spin_step(node, x, y, gradient=True, duration_s=0.055)
            if node.state == "REGION_HOMING":
                break
        wait_for_state(node, "REGION_HOMING", x, y, 2.0)

        # High SNR must request visual confirmation without publishing SUCCESS.
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            spin_step(node, x, y, snr=True, duration_s=0.05)
            if any(value for _, value in node.search_events):
                break
        search_true = [stamp for stamp, value in node.search_events if value]
        if not search_true:
            raise AssertionError("SNR success did not publish vision_search_active")
        if "SUCCESS" in node.state_history:
            raise AssertionError("handoff-enabled controller entered standalone SUCCESS")

        # target_confirmed must cause neutral RC before vision grant.
        confirm_time = time.monotonic()
        node.confirm_pub.publish(Bool(data=True))
        deadline = confirm_time + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_odom(x, y)
            rclpy.spin_once(node, timeout_sec=0.02)
            if any(value for _, value in node.grant_events):
                break
        grant_true = [stamp for stamp, value in node.grant_events if value]
        neutral_after_confirm = [
            stamp
            for stamp, channels in node.rc_events
            if stamp >= confirm_time and controlled_channels_are_neutral(channels)
        ]
        if not neutral_after_confirm:
            raise AssertionError("no neutral acoustic RC after target_confirmed")
        if not grant_true:
            raise AssertionError("vision_control_granted was not published")
        if min(neutral_after_confirm) > min(grant_true):
            raise AssertionError("vision grant preceded acoustic neutral RC")
        if node.state != "HANDOFF_COMPLETE":
            raise AssertionError(
                f"handoff did not complete: state={node.state} "
                f"history={node.state_history}"
            )

        print(
            "hydrophone_vision_handoff_contract=PASS "
            f"states={','.join(node.state_history)} "
            f"neutral_to_grant_ms="
            f"{(min(grant_true) - min(neutral_after_confirm)) * 1000.0:.1f}"
        )
        return 0
    finally:
        stop_process(process)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
