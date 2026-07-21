"""ROS2 bag loading facade for GUI RC override replay."""

from __future__ import annotations

from pathlib import Path

from .config import RC_REPLAY_TOPIC
from .models import RcReplaySample
from .rc_replay_decode import decode_replay_samples
from .rc_replay_path import rc_replay_bag_uri
from .rc_replay_rosbag import open_replay_reader, require_replay_runtime, validate_replay_topic


def load_rc_override_replay(path_text: str) -> list[RcReplaySample]:
    rosbag2_py, deserialize_message, override_rc_in_type = require_replay_runtime()
    bag_uri = rc_replay_bag_uri(path_text)
    if not Path(bag_uri).exists():
        raise RuntimeError(f"bag path does not exist: {bag_uri}")

    reader = open_replay_reader(bag_uri, rosbag2_py)
    validate_replay_topic(reader)
    samples = decode_replay_samples(
        reader,
        deserialize_message=deserialize_message,
        override_rc_in_type=override_rc_in_type,
    )
    if not samples:
        raise RuntimeError(f"{RC_REPLAY_TOPIC} has no messages")
    return samples


__all__ = ["load_rc_override_replay", "rc_replay_bag_uri"]
