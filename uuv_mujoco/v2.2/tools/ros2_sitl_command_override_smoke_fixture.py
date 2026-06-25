"""Fixtures for SITL command override smoke checks."""

from __future__ import annotations


class FakeLock:
    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        return False


class FakeTransport:
    def __init__(self) -> None:
        self.marked: list[str] = []

    def mark_sensor_replay_input_seen(self, topic: str) -> None:
        self.marked.append(topic)


class FakeBridge:
    def __init__(self) -> None:
        self._sitl_transport = FakeTransport()
        self._sitl_transport_lock = FakeLock()
        self.replay_calls: list[tuple[list[int], str]] = []
        self.arm_calls: list[tuple[bool, str]] = []
        self.mode_calls: list[tuple[str, str]] = []

    def _parse_command_override_payload(self, text: str):
        return {
            "sensor_replay_start": True,
            "replay_rcout": [1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1999],
            "arm": "true",
            "mode": text,
        }

    def _parse_command_bool(self, value, default: bool = False) -> bool:
        if value is None:
            return default
        return str(value).strip().lower() in {"true", "1", "yes", "arm", "armed"}

    def _handle_replay_rcout_channels(self, channels, *, source: str) -> None:
        self.replay_calls.append((list(channels), source))

    def _forward_arm_request(self, value: bool, topic: str) -> bool:
        self.arm_calls.append((bool(value), topic))
        return True

    def _forward_mode_request(self, mode: str, topic: str) -> bool:
        self.mode_calls.append((mode, topic))
        return True


__all__ = ["FakeBridge"]
