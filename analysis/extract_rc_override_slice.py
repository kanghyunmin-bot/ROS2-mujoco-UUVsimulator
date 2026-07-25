#!/usr/bin/env python3
"""Extract a timestamp window of /mavros/rc/override into a small ROS 2 bag."""

from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path

import rosbag2_py


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--start", type=float, required=True, help="seconds from source bag start")
    parser.add_argument("--end", type=float, required=True, help="seconds from source bag start")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.output.exists():
        raise SystemExit(f"output already exists: {args.output}")
    if args.end <= args.start:
        raise SystemExit("--end must be greater than --start")

    connection = sqlite3.connect(args.source)
    bag_start_ns = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    topic = connection.execute(
        "SELECT id, name, type, serialization_format FROM topics WHERE name = '/mavros/rc/override'"
    ).fetchone()
    if topic is None:
        raise SystemExit("/mavros/rc/override not found")
    topic_id, topic_name, topic_type, serialization_format = topic
    start_ns = bag_start_ns + round(args.start * 1.0e9)
    end_ns = bag_start_ns + round(args.end * 1.0e9)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic_name,
            type=topic_type,
            serialization_format=serialization_format,
        )
    )

    count = 0
    first_ns = None
    last_ns = None
    query = """
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ? AND timestamp >= ? AND timestamp <= ?
        ORDER BY timestamp
    """
    for timestamp_ns, data in connection.execute(query, (topic_id, start_ns, end_ns)):
        writer.write(topic_name, bytes(data), timestamp_ns)
        count += 1
        first_ns = timestamp_ns if first_ns is None else first_ns
        last_ns = timestamp_ns
    connection.close()

    if count == 0:
        raise SystemExit("selected interval contains no RC override messages")
    print(f"messages={count}")
    print(f"source_offsets_s={(first_ns - bag_start_ns) * 1.0e-9:.6f}..{(last_ns - bag_start_ns) * 1.0e-9:.6f}")
    print(f"message_span_s={(last_ns - first_ns) * 1.0e-9:.6f}")


if __name__ == "__main__":
    main()
