"""Logging helpers for replay RCOUT plant input callbacks."""

from __future__ import annotations

import time


REPLAY_RCOUT_LOG_PERIOD_S = 3.0


def warn_replay_rcout_ignored(bridge: object) -> None:
    now = time.monotonic()
    if now - bridge._mavros_last_rc_override_warn_wall <= REPLAY_RCOUT_LOG_PERIOD_S:
        return
    print(
        "[ros2] replay RCOUT override ignored: no SITL injector or direct plant handler is registered",
        flush=True,
    )
    bridge._mavros_last_rc_override_warn_wall = now


def warn_replay_rcout_rejected(bridge: object, exc: Exception) -> None:
    now = time.monotonic()
    if now - bridge._mavros_last_rc_override_warn_wall <= REPLAY_RCOUT_LOG_PERIOD_S:
        return
    print(f"[ros2] replay RCOUT override rejected: {exc}", flush=True)
    bridge._mavros_last_rc_override_warn_wall = now


def log_replay_rcout_accepted(bridge: object, channels: list[int], *, source: str) -> None:
    now = time.monotonic()
    first_row = bridge._replay_rcout_count == 1
    if not first_row and now - bridge._replay_rcout_last_log_wall <= REPLAY_RCOUT_LOG_PERIOD_S:
        return
    print(
        f"[ros2] replay RCOUT accepted via {source}: "
        f"count={bridge._replay_rcout_count} pwm[1..8]={tuple(channels[:8])}",
        flush=True,
    )
    bridge._replay_rcout_last_log_wall = now


__all__ = [
    "REPLAY_RCOUT_LOG_PERIOD_S",
    "log_replay_rcout_accepted",
    "warn_replay_rcout_ignored",
    "warn_replay_rcout_rejected",
]
