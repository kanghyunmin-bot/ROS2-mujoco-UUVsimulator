"""Channel extraction helpers for replay RCOUT plant input."""

from __future__ import annotations


REPLAY_RCOUT_CHANNEL_COUNT = 8


def first_replay_rcout_channels(channels: list[int]) -> list[int]:
    return [int(value) for value in channels[:REPLAY_RCOUT_CHANNEL_COUNT]]


def replay_rcout_channels_from_msg(msg) -> list[int]:
    return first_replay_rcout_channels(list(getattr(msg, "channels", [])))


__all__ = ["REPLAY_RCOUT_CHANNEL_COUNT", "first_replay_rcout_channels", "replay_rcout_channels_from_msg"]
