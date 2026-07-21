"""MAVROS replay/override bookkeeping defaults."""

from __future__ import annotations

from typing import Callable, Optional


def configure_mavros_replay_state(bridge: object) -> None:
    bridge._mavros_last_rc_override = None
    bridge._mavros_last_rc_override_warn_wall = -1.0
    bridge._mavros_last_rc_out = None
    bridge._replay_rcout_handler: Optional[Callable[[list[int]], None]] = None
    bridge._replay_rcout_count = 0
    bridge._replay_rcout_last_log_wall = -1.0


__all__ = ["configure_mavros_replay_state"]
