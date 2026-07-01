"""RC override replay runtime for the web GUI."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

from .config import DEFAULT_RC_REPLAY_BAG
from .rc_replay_loader import load_rc_override_replay
from .replay_format import format_replay_time
from .replay_time_math import normalized_replay_rate, replay_sample_index_for_time, replay_time_label


class WebRcReplayManager:
    """Loads and publishes RC override replay samples without Tk state."""

    def __init__(self, node: Any, release_rc: Callable[[], None]) -> None:
        self.node = node
        self._release_rc = release_rc
        self.path = str(DEFAULT_RC_REPLAY_BAG)
        self.rate = "1.0"
        self.status = "replay: unloaded"
        self.time_label = "00:00.0 / 00:00.0"
        self._samples = []
        self._duration_s = 0.0
        self._position_s = 0.0
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._seek_time_s: float | None = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()

    def status_payload(self) -> dict[str, Any]:
        return {
            "rc_replay_path": self.path,
            "rc_replay_rate": self.rate,
            "rc_replay_status": self.status,
            "rc_replay_time": self.time_label,
            "rc_replay_duration_s": self._duration_s,
            "rc_replay_position_s": self._position_s,
        }

    def load(self, path: str | None = None, rate: str | None = None) -> dict[str, Any]:
        if self.running():
            self._set_status("replay: stop current playback before loading")
            return {"status": self.status}
        if path is not None:
            self.path = str(path).strip() or str(DEFAULT_RC_REPLAY_BAG)
        if rate is not None:
            self.rate = f"{normalized_replay_rate(rate):g}"
        try:
            samples = load_rc_override_replay(self.path)
        except Exception as exc:
            self._samples = []
            self._duration_s = 0.0
            self._set_position(0.0)
            self._set_status(f"replay load failed: {exc}")
            return {"status": self.status}
        self._samples = samples
        self._duration_s = samples[-1].time_s if samples else 0.0
        self._set_position(0.0)
        self._set_status(f"replay: loaded {len(samples)} samples, duration={format_replay_time(self._duration_s)}")
        return {"status": self.status, "samples": len(samples), "duration_s": self._duration_s}

    def start(self, path: str | None = None, rate: str | None = None) -> dict[str, Any]:
        if self.running():
            self._set_status("replay: already running")
            return {"status": self.status}
        if path is not None or rate is not None or not self._samples:
            loaded = self.load(path=path, rate=rate)
            if not self._samples:
                return loaded
        self._release_rc()
        self._stop_event.clear()
        self._pause_event.clear()
        self._thread = threading.Thread(target=self._run, name="uuv-web-rc-replay", daemon=True)
        self._thread.start()
        self._set_status("replay: running")
        return {"status": self.status}

    def toggle_pause(self) -> dict[str, Any]:
        if not self.running():
            self._set_status("replay: not running")
            return {"status": self.status}
        if self._pause_event.is_set():
            self._pause_event.clear()
            self._set_status("replay: running")
        else:
            self._pause_event.set()
            self._set_status("replay: paused")
        return {"status": self.status, "paused": self._pause_event.is_set()}

    def stop(self) -> dict[str, Any]:
        running = self.running()
        self._stop_event.set()
        self._pause_event.clear()
        if self._thread is not None and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        if not running:
            self._set_status("replay: stopped")
        return {"status": self.status}

    def seek(self, time_s: float) -> dict[str, Any]:
        time_s = max(0.0, min(float(time_s), max(self._duration_s, 0.0)))
        with self._lock:
            self._seek_time_s = time_s
        self._set_position(time_s)
        self._set_status(f"replay: seek {format_replay_time(time_s)}")
        return {"status": self.status, "time_s": time_s}

    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        samples = list(self._samples)
        if not samples:
            self._set_status("replay: no samples")
            return
        rate = normalized_replay_rate(self.rate)
        self.rate = f"{rate:g}"
        idx = replay_sample_index_for_time(samples, self._duration_s, self._position_s)
        base_sample_time = samples[idx].time_s
        base_wall = time.monotonic()
        try:
            while idx < len(samples) and not self._stop_event.is_set():
                seek_time = self._take_seek()
                if seek_time is not None:
                    idx = replay_sample_index_for_time(samples, self._duration_s, seek_time)
                    base_sample_time = samples[idx].time_s
                    base_wall = time.monotonic()
                    continue
                if self._pause_event.is_set():
                    idx = self._wait_while_paused(samples, idx)
                    base_sample_time = samples[idx].time_s
                    base_wall = time.monotonic()
                    continue
                sample = samples[idx]
                target_wall = base_wall + max(sample.time_s - base_sample_time, 0.0) / rate
                if not self._wait_until(target_wall):
                    break
                seek_time = self._take_seek()
                if seek_time is not None:
                    idx = replay_sample_index_for_time(samples, self._duration_s, seek_time)
                    base_sample_time = samples[idx].time_s
                    base_wall = time.monotonic()
                    continue
                if self._pause_event.is_set():
                    continue
                if not self.node.publish_rc_channels(sample.channels):
                    self._set_status("replay failed: rc override publisher unavailable")
                    break
                self._set_position(sample.time_s)
                if idx % 10 == 0:
                    self._set_status(
                        f"replay: {idx + 1}/{len(samples)}  {format_replay_time(sample.time_s)}  rate={rate:g}x"
                    )
                idx += 1
            if self._stop_event.is_set():
                self._set_status("replay: stopped")
            elif idx >= len(samples):
                self._set_position(self._duration_s)
                self._set_status("replay: finished")
        finally:
            self._stop_event.clear()
            self._pause_event.clear()

    def _wait_while_paused(self, samples: list[Any], idx: int) -> int:
        self._set_status(f"replay: paused at {format_replay_time(self._position_s)}")
        while self._pause_event.is_set() and not self._stop_event.is_set():
            seek_time = self._take_seek()
            if seek_time is not None:
                idx = replay_sample_index_for_time(samples, self._duration_s, seek_time)
                self._set_position(samples[idx].time_s)
            time.sleep(0.05)
        return idx

    def _wait_until(self, target_wall: float) -> bool:
        while not self._stop_event.is_set():
            with self._lock:
                has_seek = self._seek_time_s is not None
            if self._pause_event.is_set() or has_seek:
                return True
            remaining = target_wall - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(remaining, 0.03))
        return False

    def _take_seek(self) -> float | None:
        with self._lock:
            value = self._seek_time_s
            self._seek_time_s = None
        return value

    def _set_position(self, time_s: float) -> None:
        self._position_s = max(0.0, min(float(time_s), max(self._duration_s, 0.0)))
        self.time_label = replay_time_label(self._position_s, self._duration_s)

    def _set_status(self, text: str) -> None:
        self.status = text
        self.node.push_event(text)


__all__ = ["WebRcReplayManager"]
