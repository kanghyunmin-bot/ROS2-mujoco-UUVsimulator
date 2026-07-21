#!/usr/bin/env python3
"""Live contract check for an external FSM using the simulator's MAVROS surface."""

from __future__ import annotations

import argparse
import json
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    import rclpy
    from audio_common_msgs.msg import AudioData
    from hit25_auv_ros2_msg.msg import CollectorState
    from mavros_msgs.msg import OverrideRCIn, State
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    class ExternalFsmProbe(Node):
        def __init__(self) -> None:
            super().__init__("external_fsm_mavros_contract_probe")
            self.counts = {
                "mavros_state": 0,
                "odometry_filtered": 0,
                "camera_compressed": 0,
                "collector_state": 0,
                "hydrophone_audio": 0,
                "rc_loopback": 0,
            }
            self.connected = False
            self.armed = False
            self.rc_message = OverrideRCIn()
            self.rc_message.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
            self.rc_pub = self.create_publisher(
                OverrideRCIn, "/mavros/rc/override", 10)
            self.create_subscription(State, "/mavros/state", self._state, 10)
            self.create_subscription(
                Odometry, "/odometry/filtered", self._count("odometry_filtered"), 10)
            self.create_subscription(
                CompressedImage,
                "/camera/camera/color/image_raw/compressed",
                self._count("camera_compressed"),
                10,
            )
            self.create_subscription(
                CollectorState, "/collector/state", self._count("collector_state"), 10)
            self.create_subscription(
                AudioData, "/audio", self._count("hydrophone_audio"), 10)
            self.create_subscription(
                OverrideRCIn, "/mavros/rc/override", self._rc_loopback, 10)
            self.create_timer(0.2, self._publish_safe_rc_probe)

        def _count(self, key: str):
            def callback(_msg) -> None:
                self.counts[key] += 1

            return callback

        def _state(self, msg: State) -> None:
            self.counts["mavros_state"] += 1
            self.connected = bool(msg.connected)
            self.armed = bool(msg.armed)

        def _publish_safe_rc_probe(self) -> None:
            # CHAN_NOCHANGE exercises the exact external-FSM publish path while
            # remaining safe even if the vehicle happens to be armed.
            self.rc_pub.publish(self.rc_message)

        def _rc_loopback(self, msg: OverrideRCIn) -> None:
            if list(msg.channels) == list(self.rc_message.channels):
                self.counts["rc_loopback"] += 1

    rclpy.init()
    node = ExternalFsmProbe()
    deadline = time.monotonic() + max(1.0, float(args.duration_s))
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        rc_subscribers = node.count_subscribers("/mavros/rc/override")
        payload = {
            "connected": node.connected,
            "armed": node.armed,
            "rc_override_subscribers": rc_subscribers,
            "counts": node.counts,
        }
        required = {
            "mavros_connected": node.connected,
            "mavros_state_rx": node.counts["mavros_state"] > 0,
            "rc_override_has_consumer": rc_subscribers > 0,
            "rc_override_tx_loopback": node.counts["rc_loopback"] > 0,
            "odometry_filtered_rx": node.counts["odometry_filtered"] > 0,
            "camera_compressed_rx": node.counts["camera_compressed"] > 0,
            "collector_state_rx": node.counts["collector_state"] > 0,
            "hydrophone_audio_rx": node.counts["hydrophone_audio"] > 0,
        }
        payload["checks"] = required
        payload["overall"] = "pass" if all(required.values()) else "fail"
        print(json.dumps(payload, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()

    return 0 if payload["overall"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
