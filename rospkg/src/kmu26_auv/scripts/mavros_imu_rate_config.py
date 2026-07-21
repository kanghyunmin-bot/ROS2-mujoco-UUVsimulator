#!/usr/bin/python3

import math
import sys
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node


class MavrosImuRateConfig(Node):
    """Request MAVLink IMU-related message rates through MAVROS."""

    MESSAGE_IDS = {
        "ATTITUDE": 30,
        "ATTITUDE_QUATERNION": 31,
        "RAW_IMU": 27,
        "HIGHRES_IMU": 105,
    }

    def __init__(self):
        super().__init__("mavros_imu_rate_config")

        self.service_name = self.declare_parameter(
            "service_name", "/mavros/set_message_interval"
        ).value
        self.state_topic = self.declare_parameter("state_topic", "/mavros/state").value
        self.wait_for_connected = bool(
            self.declare_parameter("wait_for_connected", True).value
        )
        self.startup_timeout_s = float(
            self.declare_parameter("startup_timeout_s", 30.0).value
        )
        self.response_timeout_s = float(
            self.declare_parameter("response_timeout_s", 5.0).value
        )
        self.retry_count = int(self.declare_parameter("retry_count", 3).value)
        self.retry_delay_s = float(self.declare_parameter("retry_delay_s", 1.0).value)

        self.requests = self._build_requests()
        self.connected = False
        self.client = self.create_client(MessageInterval, self.service_name)
        self.state_sub = self.create_subscription(
            State, self.state_topic, self._state_cb, 10
        )

    def _build_requests(self):
        requests = []
        rate_params = [
            ("ATTITUDE", "attitude_rate_hz", 50.0),
            ("RAW_IMU", "raw_imu_rate_hz", 50.0),
            ("ATTITUDE_QUATERNION", "attitude_quaternion_rate_hz", -1.0),
            ("HIGHRES_IMU", "highres_imu_rate_hz", -1.0),
        ]

        for label, param_name, default_rate in rate_params:
            rate_hz = float(self.declare_parameter(param_name, default_rate).value)
            if rate_hz < 0.0:
                continue
            if not math.isfinite(rate_hz):
                raise ValueError(f"{param_name} must be finite")
            requests.append((label, self.MESSAGE_IDS[label], rate_hz))

        return requests

    def _state_cb(self, msg):
        self.connected = msg.connected

    def _wait_for_service(self, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            timeout_s = min(0.5, max(0.0, deadline - time.monotonic()))
            if self.client.wait_for_service(timeout_sec=timeout_s):
                return True

        self.get_logger().error(f"Service not available: {self.service_name}")
        return False

    def _wait_for_fcu_connected(self, deadline):
        if not self.wait_for_connected:
            return True

        while rclpy.ok() and time.monotonic() < deadline:
            if self.connected:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(f"Timed out waiting for FCU connection on {self.state_topic}")
        return False

    def _call_interval(self, label, message_id, rate_hz):
        request = MessageInterval.Request()
        request.message_id = message_id
        request.message_rate = float(rate_hz)

        self.get_logger().info(
            f"Requesting {label} (msgid {message_id}) at {rate_hz:.1f} Hz"
        )

        future = self.client.call_async(request)
        deadline = time.monotonic() + self.response_timeout_s

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error(f"Timed out setting {label} message interval")
            return False

        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(f"FCU rejected {label} message interval request")
            return False

        return True

    def configure(self):
        if not self.requests:
            self.get_logger().warn("No message interval requests configured")
            return True

        deadline = time.monotonic() + self.startup_timeout_s
        if not self._wait_for_service(deadline):
            return False
        if not self._wait_for_fcu_connected(deadline):
            return False

        for attempt in range(1, self.retry_count + 1):
            all_ok = True
            for label, message_id, rate_hz in self.requests:
                all_ok = self._call_interval(label, message_id, rate_hz) and all_ok

            if all_ok:
                self.get_logger().info("MAVROS IMU message rates configured")
                return True

            if attempt < self.retry_count:
                self.get_logger().warn(
                    f"Retrying MAVROS IMU rate configuration "
                    f"({attempt}/{self.retry_count})"
                )
                retry_deadline = time.monotonic() + self.retry_delay_s
                while rclpy.ok() and time.monotonic() < retry_deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = MavrosImuRateConfig()

    try:
        success = node.configure()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
