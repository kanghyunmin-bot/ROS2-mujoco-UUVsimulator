#!/usr/bin/env python3
"""Remove the 20 Hz partial-release publisher from an RC-only ROS 2 bag."""

from __future__ import annotations

import argparse
from pathlib import Path

import rosbag2_py
from mavros_msgs.msg import OverrideRCIn
from rclpy.serialization import deserialize_message


def is_competing_partial_release(message: OverrideRCIn) -> bool:
    """Match the observed vision-controller idle frame, and nothing broader."""
    channels = [int(value) for value in message.channels]
    if len(channels) < 8:
        return False
    controlled = (2, 3, 4)  # MAVROS channels 3, 4, 5
    return all(channels[index] == OverrideRCIn.CHAN_RELEASE for index in controlled) and all(
        channels[index] == OverrideRCIn.CHAN_NOCHANGE
        for index in range(len(channels))
        if index not in controlled
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"output already exists: {args.output}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.source), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/mavros/rc/override",
            type="mavros_msgs/msg/OverrideRCIn",
            serialization_format="cdr",
        )
    )

    kept = 0
    removed = 0
    first_timestamp = None
    last_timestamp = None
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic != "/mavros/rc/override":
            continue
        message = deserialize_message(raw, OverrideRCIn)
        if is_competing_partial_release(message):
            removed += 1
            continue
        writer.write(topic, raw, timestamp)
        kept += 1
        first_timestamp = timestamp if first_timestamp is None else first_timestamp
        last_timestamp = timestamp

    if kept == 0:
        raise SystemExit("filter removed every RC message")
    print(f"kept={kept}")
    print(f"removed_partial_release={removed}")
    print(f"kept_span_s={(last_timestamp - first_timestamp) * 1.0e-9:.6f}")


if __name__ == "__main__":
    main()
