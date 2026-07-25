"""State mutation helpers for controller-parity sensor replay."""

from __future__ import annotations

from bridge.sitl_replay import SensorReplayFrame


def _remember_sensor_replay_frame(self, frame: SensorReplayFrame) -> SensorReplayFrame:
    self._sensor_replay_current_frame = frame
    return frame


def mark_sensor_replay_input_seen(self, source: str = "external") -> None:
    """Start controller-parity sensor replay without forwarding RC."""
    if self._sensor_replay_rc_seen:
        return
    self._sensor_replay_rc_seen = True
    # The JSON servo frame counter can remain on the same value across the
    # external RC-start trigger. The immediate-reply scheduler must handle the
    # first post-trigger packet even if its frame_count matches the bootstrap
    # packet sent before RC was seen.
    self._sensor_replay_immediate_last_frame_count = None
    print(
        "[sitl_transport] controller-parity sensor replay input trigger "
        f"received via {source}",
        flush=True,
    )


__all__ = ["_remember_sensor_replay_frame", "mark_sensor_replay_input_seen"]
