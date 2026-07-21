"""ROS bag runtime helpers for GUI RC override replay loading."""

from __future__ import annotations

from .config import RC_REPLAY_TOPIC


def require_replay_runtime():
    from .runtime import (
        HAVE_MAVROS_MSGS,
        HAVE_ROSBAG2_PY,
        OverrideRCIn,
        deserialize_message,
        rosbag2_py,
    )

    if not HAVE_MAVROS_MSGS:
        raise RuntimeError("mavros_msgs is not available in this Python environment")
    if not HAVE_ROSBAG2_PY or rosbag2_py is None or deserialize_message is None:
        raise RuntimeError("rosbag2_py is not available in this Python environment")
    return rosbag2_py, deserialize_message, OverrideRCIn


def open_replay_reader(bag_uri: str, rosbag2_py):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def validate_replay_topic(reader) -> None:
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if RC_REPLAY_TOPIC not in topics:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} not found in bag")
    if topics[RC_REPLAY_TOPIC] != "mavros_msgs/msg/OverrideRCIn":
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has unexpected type: {topics[RC_REPLAY_TOPIC]}")


__all__ = ["open_replay_reader", "require_replay_runtime", "validate_replay_topic"]
