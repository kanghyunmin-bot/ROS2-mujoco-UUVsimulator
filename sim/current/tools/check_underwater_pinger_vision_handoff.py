#!/usr/bin/env python3
"""ROS contract test for standalone and acoustic-granted underwater vision control."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Float64, String


TARGET_ID = "course_buoy_pinger_white_1_float"


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ContractProbe(Node):
    def __init__(self) -> None:
        super().__init__("underwater_pinger_vision_contract_probe")
        self.search_pub = self.create_publisher(
            Bool, "/homing/vision_search_active", latched_qos()
        )
        self.grant_pub = self.create_publisher(
            Bool, "/homing/vision_control_granted", latched_qos()
        )
        self.depth_pub = self.create_publisher(Float64, "/depth", 10)
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, "/vision/buoy_bbox", 10
        )
        self.status_pub = self.create_publisher(
            String, "/mujoco/course_buoys/status", 10
        )

        self.state = ""
        self.state_history: list[str] = []
        self.rc_frames: list[tuple[float, list[int]]] = []
        self.target_confirmed = False
        self.detached = False
        self.detached_messages = 0
        self.success = False
        self.create_subscription(
            String, "/mission/state", self._on_state, latched_qos()
        )
        self.create_subscription(
            OverrideRCIn, "/mission/rc_command", self._on_rc, 10
        )
        self.create_subscription(
            Bool, "/vision/target_confirmed", self._on_confirmed, latched_qos()
        )
        self.create_subscription(
            Bool, "/vision/pinger_detached", self._on_detached, latched_qos()
        )
        self.create_subscription(
            Bool, "/mission/success", self._on_success, latched_qos()
        )

    def reset_observations(self) -> None:
        self.state = ""
        self.state_history.clear()
        self.rc_frames.clear()
        self.target_confirmed = False
        self.detached = False
        self.detached_messages = 0
        self.success = False

    def _on_state(self, msg: String) -> None:
        self.state = str(msg.data)
        if not self.state_history or self.state_history[-1] != self.state:
            self.state_history.append(self.state)

    def _on_rc(self, msg: OverrideRCIn) -> None:
        self.rc_frames.append((time.monotonic(), list(msg.channels)))

    def _on_confirmed(self, msg: Bool) -> None:
        self.target_confirmed = bool(msg.data)

    def _on_detached(self, msg: Bool) -> None:
        self.detached_messages += 1
        self.detached = bool(msg.data)

    def _on_success(self, msg: Bool) -> None:
        self.success = bool(msg.data)

    def publish_inputs(
        self, *, search: bool, grant: bool, detached: bool, include_stick: bool
    ) -> None:
        self.search_pub.publish(Bool(data=search))
        self.grant_pub.publish(Bool(data=grant))
        self.depth_pub.publish(Float64(data=8.65))
        self.bbox_pub.publish(
            detection(
                class_id=0,
                confidence=0.92,
                center_x=320.0,
                center_y=240.0,
                width=420.0,
                height=360.0,
            )
        )
        if include_stick:
            self.bbox_pub.publish(
                detection(
                    class_id=1,
                    confidence=0.90,
                    center_x=192.0,
                    center_y=336.0,
                    width=80.0,
                    height=180.0,
                )
            )
        status = String()
        status.data = json.dumps(
            {
                "buoys": [
                    {
                        "id": TARGET_ID,
                        "has_magnet": True,
                        "detached": detached,
                    }
                ]
            }
        )
        self.status_pub.publish(status)


def detection(
    *,
    class_id: int,
    confidence: float,
    center_x: float,
    center_y: float,
    width: float,
    height: float,
) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [
        time.monotonic(),
        1.0,
        float(class_id),
        confidence,
        center_x,
        center_y,
        width,
        height,
        640.0,
        480.0,
    ]
    return msg


def start_control_launch(*, force_control_grant: bool) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        [
            "ros2",
            "launch",
            "auv_buoy_vision_control",
            "underwater_pinger_vision_control.launch.py",
            "use_sim_time:=false",
            f"force_control_grant:={'true' if force_control_grant else 'false'}",
        ],
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def spin_for(
    node: ContractProbe,
    duration_s: float,
    *,
    search: bool,
    grant: bool,
    detached: bool,
    include_stick: bool,
) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_inputs(
            search=search,
            grant=grant,
            detached=detached,
            include_stick=include_stick,
        )
        rclpy.spin_once(node, timeout_sec=0.03)


def check_forced_grant(node: ContractProbe) -> None:
    node.reset_observations()
    process = start_control_launch(force_control_grant=True)
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_inputs(
                search=False,
                grant=False,
                detached=False,
                include_stick=False,
            )
            rclpy.spin_once(node, timeout_sec=0.03)
            if (
                node.state in {"SEARCH", "APPROACH_BUOY"}
                and node.rc_frames
                and node.detached_messages > 0
            ):
                break
        if process.poll() is not None:
            raise AssertionError("standalone control launch exited early")
        if not node.rc_frames:
            raise AssertionError("forced grant did not produce vision RC")
        if "SEARCH" not in node.state_history:
            raise AssertionError(
                f"forced grant did not enter SEARCH: {node.state_history}"
            )
    finally:
        stop_process(process)
        spin_for(
            node,
            0.3,
            search=False,
            grant=False,
            detached=False,
            include_stick=False,
        )


def check_handoff_and_physical_success(node: ContractProbe) -> None:
    node.reset_observations()
    process = start_control_launch(force_control_grant=False)
    try:
        deadline = time.monotonic() + 7.0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_inputs(
                search=True,
                grant=False,
                detached=False,
                include_stick=True,
            )
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.state == "WAIT_CONTROL_GRANT" and node.target_confirmed:
                break
        if node.state != "WAIT_CONTROL_GRANT" or not node.target_confirmed:
            raise AssertionError(
                "vision did not complete search->target_confirmed: "
                f"{node.state_history}"
            )
        if node.rc_frames:
            raise AssertionError(
                f"vision published {len(node.rc_frames)} RC frame(s) before grant"
            )

        grant_time = time.monotonic()
        deadline = grant_time + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            physical_detached = node.state == "VERIFY_RELEASE"
            node.publish_inputs(
                search=True,
                grant=True,
                detached=physical_detached,
                include_stick=True,
            )
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.state == "COMPLETE" and node.success and node.detached:
                break
        if process.poll() is not None:
            raise AssertionError("handoff control launch exited early")
        required_states = {
            "TARGET_CONFIRM",
            "WAIT_CONTROL_GRANT",
            "APPROACH_BUOY",
            "ALIGN_STICK",
            "INSERT_FORK",
            "DETACH",
            "BACKOFF",
            "VERIFY_RELEASE",
            "COMPLETE",
        }
        missing = required_states.difference(node.state_history)
        if missing:
            raise AssertionError(
                f"physical mission states missing={sorted(missing)} "
                f"history={node.state_history}"
            )
        if not (node.detached and node.success and node.state == "COMPLETE"):
            raise AssertionError(
                "detached=true did not produce physical COMPLETE: "
                f"detached={node.detached} success={node.success} "
                f"state={node.state}"
            )
        if not any(timestamp >= grant_time for timestamp, _ in node.rc_frames):
            raise AssertionError("vision did not publish RC after grant")
    finally:
        stop_process(process)


def main() -> int:
    rclpy.init()
    node = ContractProbe()
    try:
        check_forced_grant(node)
        check_handoff_and_physical_success(node)
        print(
            "underwater_pinger_vision_contract=PASS "
            f"states={','.join(node.state_history)} "
            f"rc_frames={len(node.rc_frames)} detached={node.detached}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
