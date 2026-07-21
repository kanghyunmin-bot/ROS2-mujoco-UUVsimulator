"""Compatibility facade for GUI RC override replay controls."""

from __future__ import annotations

from .replay_controls import (
    _browse_rc_replay_bag,
    _load_rc_replay,
    _start_rc_replay,
    _stop_rc_replay,
    _toggle_rc_replay_pause,
)
from .replay_status import (
    _rc_replay_running,
    _set_rc_replay_pause_button,
    _set_rc_replay_status,
)
from .replay_timeline import (
    _consume_rc_replay_seek,
    _event_to_rc_replay_time,
    _on_rc_replay_slider_changed,
    _on_rc_replay_slider_motion,
    _on_rc_replay_slider_press,
    _on_rc_replay_slider_release,
    _rc_replay_rate,
    _rc_replay_sample_index_for_time,
    _request_rc_replay_seek,
    _set_rc_replay_position,
    _set_replay_slider_from_event,
    _update_rc_replay_time_label,
)
from .replay_worker import _run_rc_replay


class RcReplayMixin:
    """Preserve the historical replay mixin API while keeping roles separated."""

    _set_rc_replay_status = _set_rc_replay_status
    _set_rc_replay_pause_button = _set_rc_replay_pause_button
    _rc_replay_running = _rc_replay_running

    _rc_replay_sample_index_for_time = _rc_replay_sample_index_for_time
    _set_rc_replay_position = _set_rc_replay_position
    _update_rc_replay_time_label = _update_rc_replay_time_label
    _event_to_rc_replay_time = _event_to_rc_replay_time
    _set_replay_slider_from_event = _set_replay_slider_from_event
    _on_rc_replay_slider_changed = _on_rc_replay_slider_changed
    _on_rc_replay_slider_press = _on_rc_replay_slider_press
    _on_rc_replay_slider_motion = _on_rc_replay_slider_motion
    _on_rc_replay_slider_release = _on_rc_replay_slider_release
    _request_rc_replay_seek = _request_rc_replay_seek
    _consume_rc_replay_seek = _consume_rc_replay_seek
    _rc_replay_rate = _rc_replay_rate

    _browse_rc_replay_bag = _browse_rc_replay_bag
    _load_rc_replay = _load_rc_replay
    _start_rc_replay = _start_rc_replay
    _toggle_rc_replay_pause = _toggle_rc_replay_pause
    _stop_rc_replay = _stop_rc_replay

    _run_rc_replay = _run_rc_replay
