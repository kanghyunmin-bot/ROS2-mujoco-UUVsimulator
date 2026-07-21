#!/usr/bin/env python3
"""Smoke-check GUI RC replay sample decoding without ROS imports."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config import RC_MESSAGE_CHANNEL_COUNT, RC_REPLAY_TOPIC  # noqa: E402
from gui.rc_replay_decode import decode_replay_samples  # noqa: E402
from gui.rc_replay_path import rc_replay_bag_uri  # noqa: E402


class FakeMsg:
    def __init__(self, channels) -> None:
        self.channels = channels


class FakeReader:
    def __init__(self, rows) -> None:
        self.rows = list(rows)
        self.index = 0

    def has_next(self) -> bool:
        return self.index < len(self.rows)

    def read_next(self):
        row = self.rows[self.index]
        self.index += 1
        return row


def deserialize_message(data, _typ):
    return FakeMsg(data)


def main() -> int:
    assert rc_replay_bag_uri("/tmp/foo.db3") == "/tmp"
    assert rc_replay_bag_uri("/tmp/bag_dir") == "/tmp/bag_dir"
    reader = FakeReader(
        [
            ("/other", [999], 1000),
            (RC_REPLAY_TOPIC, [1500, 1600], 1_000_000_000),
            (RC_REPLAY_TOPIC, [1100] * 20, 1_250_000_000),
        ]
    )
    samples = decode_replay_samples(reader, deserialize_message=deserialize_message, override_rc_in_type=object)
    assert len(samples) == 2
    assert samples[0].time_s == 0.0
    assert samples[1].time_s == 0.25
    assert len(samples[0].channels) == RC_MESSAGE_CHANNEL_COUNT
    assert samples[0].channels[:2] == (1500, 1600)
    assert len(samples[1].channels) == RC_MESSAGE_CHANNEL_COUNT
    assert samples[1].channels[-1] == 1100
    print("gui_rc_replay_decode=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
