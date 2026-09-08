#!/usr/bin/env python3
"""Exercise the physical IMX219 node with delayed timestamped GStreamer video."""

from __future__ import annotations

import os
import statistics
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


PIPELINE = (
    "videotestsrc is-live=true do-timestamp=true pattern=black "
    "! video/x-raw,format=BGR,width=64,height=48,framerate=10/1 "
    "! identity sleep-time=80000 "
    "! appsink name=imx219_sink max-buffers=1 drop=true sync=false"
)


def main() -> int:
    environment = dict(os.environ)
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "auv_imx219_camera",
            "imx219_camera_node",
            "--ros-args",
            "-r",
            "__ns:=/imx219_timestamp_test",
            "-p",
            f"pipeline:={PIPELINE}",
            "-p",
            "width:=64",
            "-p",
            "height:=48",
            "-p",
            "framerate:=10",
            "-p",
            "timestamp_source:=gstreamer_pts",
            "-p",
            "max_capture_age_ms:=1000",
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    rclpy.init()
    node = rclpy.create_node("imx219_capture_timestamp_e2e")
    image_stamps: list[int] = []
    info_stamps: set[int] = set()
    image_ages_s: list[float] = []

    def on_image(message: Image) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        image_stamps.append(stamp_ns)
        image_ages_s.append((node.get_clock().now().nanoseconds - stamp_ns) * 1e-9)
        assert message.encoding == "bgr8"
        assert message.width == 64 and message.height == 48

    def on_info(message: CameraInfo) -> None:
        info_stamps.add(
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    node.create_subscription(
        Image,
        "/imx219_timestamp_test/image_raw",
        on_image,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        CameraInfo,
        "/imx219_timestamp_test/camera_info",
        on_info,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 12.0
    try:
        while len(image_stamps) < 6 and time.monotonic() < deadline:
            if process.poll() is not None:
                output = process.stdout.read() if process.stdout is not None else ""
                raise RuntimeError(f"IMX219 test node exited early:\n{output}")
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(image_stamps) >= 6, "timed out waiting for physical-driver test images"
        checked_image_stamps = list(image_stamps[:6])
        info_deadline = time.monotonic() + 1.0
        while not set(checked_image_stamps).issubset(info_stamps) and time.monotonic() < info_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert all(
            second > first
            for first, second in zip(checked_image_stamps, checked_image_stamps[1:])
        )
        assert set(checked_image_stamps).issubset(
            info_stamps
        ), "Image and CameraInfo stamps differ"
        median_age_s = statistics.median(image_ages_s[1:])
        assert 0.04 <= median_age_s <= 0.6, (
            f"capture stamp age {median_age_s:.6f}s does not reflect the injected 80ms delay"
        )
        print(
            "imx219_capture_timestamp_e2e=PASS "
            f"frames={len(image_stamps)} median_capture_age_s={median_age_s:.6f}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


if __name__ == "__main__":
    raise SystemExit(main())
