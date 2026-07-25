"""Replay-RCOU command override handling."""

from __future__ import annotations


def _replay_rcout_channels_from_payload(payload: dict[str, object]) -> list[int] | None:
    replay_rcout = payload.get("replay_rcout", payload.get("rcout"))
    if replay_rcout is None:
        return None
    try:
        return [int(v) for v in list(replay_rcout)[:8]]
    except Exception:
        return []


def _handle_replay_rcout(self, payload: dict[str, object]) -> None:
    channels = _replay_rcout_channels_from_payload(payload)
    if channels is None:
        return
    self._handle_replay_rcout_channels(channels, source="command_override_replay_rcout")


__all__ = ["_handle_replay_rcout", "_replay_rcout_channels_from_payload"]
