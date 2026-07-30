#!/usr/bin/env python3
"""Attach acquisition timestamps to MuJoCo PCM for the real SNR pipeline."""

from __future__ import annotations

import argparse

import rclpy
from audio_common_msgs.msg import AudioData, AudioDataStamped
from rclpy.node import Node


class AudioStampedRelay(Node):
    def __init__(
        self,
        *,
        input_topic: str,
        output_topic: str,
        sample_rate: int,
        channels: int,
        bytes_per_sample: int,
    ) -> None:
        super().__init__("uuv_audio_stamped_relay")
        self._sample_rate = sample_rate
        self._frame_size = channels * bytes_per_sample
        self._publisher = self.create_publisher(AudioDataStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            AudioData, input_topic, self._on_audio, 10
        )
        self.get_logger().info(
            "Relaying %s -> %s at %d Hz, %d channels, %d-byte samples"
            % (
                input_topic,
                output_topic,
                sample_rate,
                channels,
                bytes_per_sample,
            )
        )

    def _on_audio(self, message: AudioData) -> None:
        frame_count = len(message.data) // self._frame_size
        duration_ns = round(frame_count * 1_000_000_000 / self._sample_rate)
        stamped = AudioDataStamped()
        # At simulator startup the first PCM block can arrive before /clock
        # has advanced by one complete block.  Clamp the acquisition start to
        # zero instead of constructing an invalid negative ROS time.
        start_ns = max(0, self.get_clock().now().nanoseconds - duration_ns)
        stamped.header.stamp.sec = start_ns // 1_000_000_000
        stamped.header.stamp.nanosec = start_ns % 1_000_000_000
        stamped.header.frame_id = "hydrophone_link"
        stamped.audio = message
        self._publisher.publish(stamped)


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/audio")
    parser.add_argument("--output-topic", default="/audio_stamped")
    parser.add_argument("--sample-rate", type=int, default=96_000)
    parser.add_argument("--channels", type=int, default=2)
    parser.add_argument("--bytes-per-sample", type=int, default=4)
    return parser.parse_known_args()


def main() -> None:
    args, ros_args = parse_args()
    if args.sample_rate <= 0 or args.channels <= 0 or args.bytes_per_sample <= 0:
        raise SystemExit("sample-rate, channels and bytes-per-sample must be positive")
    rclpy.init(args=ros_args)
    node = AudioStampedRelay(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        sample_rate=args.sample_rate,
        channels=args.channels,
        bytes_per_sample=args.bytes_per_sample,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
