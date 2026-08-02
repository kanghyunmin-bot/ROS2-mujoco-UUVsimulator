#!/usr/bin/env python3
"""Stop standalone pinger homing as soon as the collector reports success."""

from __future__ import annotations

import time

import rclpy
try:
    from auv_msg.msg import CollectorState
except ImportError:  # Compatibility with pre-2026 installed workspaces.
    from hit25_auv_ros2_msg.msg import CollectorState
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class PingerCaptureGuard(Node):
    def __init__(self) -> None:
        super().__init__("pinger_homing_capture_guard")
        self._captured = False
        self._target_id = ""
        self.create_subscription(CollectorState, "/collector/state", self._on_state, 10)
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")

    def _on_state(self, msg: CollectorState) -> None:
        target_id = str(msg.target_id).strip()
        if "pinger" not in target_id.lower():
            return
        if msg.detached or msg.captured or msg.netted:
            self._captured = True
            self._target_id = target_id

    def wait_for_capture(self) -> None:
        while rclpy.ok() and not self._captured:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._captured:
            return
        self.get_logger().info(
            f"pinger capture confirmed target={self._target_id or 'unknown'}; stopping thrust"
        )
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not self._arm_client.wait_for_service(timeout_sec=0.1):
            if time.monotonic() >= deadline:
                return
        request = CommandBool.Request()
        request.value = False
        future = self._arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main() -> int:
    rclpy.init()
    node = PingerCaptureGuard()
    try:
        node.wait_for_capture()
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
