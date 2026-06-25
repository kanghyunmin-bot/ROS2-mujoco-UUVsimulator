"""Replay RCOUT plant-input callbacks for Ros2Bridge."""

from __future__ import annotations

from .ros2_replay_rcout_channels import (
    REPLAY_RCOUT_CHANNEL_COUNT,
    first_replay_rcout_channels,
    replay_rcout_channels_from_msg,
)
from .ros2_replay_rcout_inject import inject_replay_rcout_channels
from .ros2_replay_rcout_log import (
    log_replay_rcout_accepted,
    warn_replay_rcout_ignored,
    warn_replay_rcout_rejected,
)


def _handle_replay_rcout_channels(self, channels: list[int], *, source: str) -> None:
    channels = first_replay_rcout_channels(channels)
    if len(channels) < REPLAY_RCOUT_CHANNEL_COUNT:
        return
    try:
        if not inject_replay_rcout_channels(self, channels, source=source):
            warn_replay_rcout_ignored(self)
            return
        self._replay_rcout_count += 1
        log_replay_rcout_accepted(self, channels, source=source)
    except Exception as exc:
        warn_replay_rcout_rejected(self, exc)


def _on_replay_rcout_override(self, msg) -> None:
    channels = replay_rcout_channels_from_msg(msg)
    self._handle_replay_rcout_channels(channels, source="replay_rcout")


__all__ = ["_handle_replay_rcout_channels", "_on_replay_rcout_override"]
