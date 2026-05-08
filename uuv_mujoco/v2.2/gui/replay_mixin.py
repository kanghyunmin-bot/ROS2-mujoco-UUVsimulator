"""RC replay controls for the MuJoCo UUV GUI."""

from __future__ import annotations

import datetime as _dt

from .config import *
from .helpers import *
from .models import ControlCommands, RcReplaySample
from .node import UuvGuiNode
from .ros_tools import *
from .runtime import *
from .widgets import VirtualJoystick

class RcReplayMixin:
    def _set_rc_replay_status(self, text: str) -> None:
        if threading.current_thread() is threading.main_thread():
            self.rc_replay_status_var.set(text)
            return
        try:
            self.root.after(0, lambda: self.rc_replay_status_var.set(text))
        except Exception:
            pass

    def _set_rc_replay_pause_button(self, text: str) -> None:
        try:
            self.root.after(0, lambda: self.rc_replay_pause_button.config(text=text))
        except Exception:
            pass

    def _rc_replay_running(self) -> bool:
        return self._rc_replay_thread is not None and self._rc_replay_thread.is_alive()

    def _rc_replay_sample_index_for_time(self, time_s: float) -> int:
        if not self._rc_replay_samples:
            return 0
        target = clamp(float(time_s), 0.0, self._rc_replay_duration_s)
        lo = 0
        hi = len(self._rc_replay_samples)
        while lo < hi:
            mid = (lo + hi) // 2
            if self._rc_replay_samples[mid].time_s < target:
                lo = mid + 1
            else:
                hi = mid
        return min(lo, len(self._rc_replay_samples) - 1)

    def _set_rc_replay_position(self, time_s: float, *, force: bool = False) -> None:
        time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))

        def apply() -> None:
            if self._rc_replay_slider_dragging and not force:
                return
            self.rc_replay_position_var.set(time_s)
            self._update_rc_replay_time_label(time_s)

        if threading.current_thread() is threading.main_thread():
            apply()
        else:
            try:
                self.root.after(0, apply)
            except Exception:
                pass

    def _update_rc_replay_time_label(self, time_s: float | None = None) -> None:
        if time_s is None:
            try:
                time_s = float(self.rc_replay_position_var.get())
            except Exception:
                time_s = 0.0
        self.rc_replay_time_var.set(
            f"{format_replay_time(time_s)} / {format_replay_time(self._rc_replay_duration_s)}"
        )

    def _event_to_rc_replay_time(self, event) -> float:
        width = max(int(self.rc_replay_slider.winfo_width()), 1)
        ratio = clamp(float(event.x) / float(width), 0.0, 1.0)
        return ratio * max(self._rc_replay_duration_s, 0.0)

    def _set_replay_slider_from_event(self, event) -> float:
        time_s = self._event_to_rc_replay_time(event)
        self.rc_replay_position_var.set(time_s)
        self._update_rc_replay_time_label(time_s)
        return time_s

    def _on_rc_replay_slider_changed(self, value: str) -> None:
        if self._rc_replay_slider_dragging:
            return
        try:
            time_s = float(value)
        except (TypeError, ValueError):
            return
        self._update_rc_replay_time_label(time_s)

    def _on_rc_replay_slider_press(self, event):
        if not self._rc_replay_samples:
            return "break"
        self._rc_replay_slider_dragging = True
        self._set_replay_slider_from_event(event)
        return "break"

    def _on_rc_replay_slider_motion(self, event):
        if not self._rc_replay_samples:
            return "break"
        self._set_replay_slider_from_event(event)
        return "break"

    def _on_rc_replay_slider_release(self, event):
        if not self._rc_replay_samples:
            self._rc_replay_slider_dragging = False
            return "break"
        time_s = self._set_replay_slider_from_event(event)
        self._rc_replay_slider_dragging = False
        self._request_rc_replay_seek(time_s)
        return "break"

    def _request_rc_replay_seek(self, time_s: float) -> None:
        time_s = clamp(float(time_s), 0.0, max(self._rc_replay_duration_s, 0.0))
        with self._rc_replay_seek_lock:
            self._rc_replay_seek_time_s = time_s
        if not self._rc_replay_running():
            self._set_rc_replay_position(time_s, force=True)
            self._set_rc_replay_status(f"replay: seek {format_replay_time(time_s)}")

    def _consume_rc_replay_seek(self) -> float | None:
        with self._rc_replay_seek_lock:
            time_s = self._rc_replay_seek_time_s
            self._rc_replay_seek_time_s = None
        return time_s

    def _browse_rc_replay_bag(self) -> None:
        initial_dir = str(DEFAULT_RC_REPLAY_BAG.parent if DEFAULT_RC_REPLAY_BAG.parent.exists() else APP_ROOT)
        selected = filedialog.askdirectory(
            parent=self.root,
            title="Select ROS2 bag directory containing /mavros/rc/override",
            initialdir=initial_dir,
        )
        if selected:
            self.rc_replay_path_var.set(selected)
            self._set_rc_replay_status("replay: selected, not loaded")

    def _load_rc_replay(self) -> bool:
        if self._rc_replay_running():
            self._set_rc_replay_status("replay: stop current playback before loading")
            return False
        try:
            samples = load_rc_override_replay(self.rc_replay_path_var.get())
        except Exception as exc:
            self._rc_replay_samples = []
            self._rc_replay_duration_s = 0.0
            self.rc_replay_slider.configure(to=1.0)
            self.rc_replay_slider.state(["disabled"])
            self._set_rc_replay_position(0.0, force=True)
            self._set_rc_replay_status(f"replay load failed: {exc}")
            return False

        self._rc_replay_samples = samples
        duration = samples[-1].time_s if samples else 0.0
        self._rc_replay_duration_s = duration
        self.rc_replay_slider.configure(to=max(duration, 1e-6))
        self.rc_replay_slider.state(["!disabled"])
        self._set_rc_replay_position(0.0, force=True)
        with self._rc_replay_seek_lock:
            self._rc_replay_seek_time_s = None
        self._set_rc_replay_status(
            f"replay loaded: {len(samples)} msgs, {format_replay_time(duration)}"
        )
        return True

    def _rc_replay_rate(self) -> float:
        try:
            rate = float(self.rc_replay_rate_var.get())
        except ValueError:
            rate = 1.0
        rate = clamp(rate, 0.1, 5.0)
        self.rc_replay_rate_var.set(f"{rate:g}")
        return rate

    def _start_rc_replay(self) -> None:
        if self._rc_replay_running():
            return
        if not self._rc_replay_samples and not self._load_rc_replay():
            return

        self.rc_override_enabled.set(False)
        self._center_rc_sticks()
        self._rc_override_prev = False

        self._rc_replay_stop_event.clear()
        self._rc_replay_pause_event.clear()
        self.rc_replay_pause_button.config(text="Pause")
        rate = self._rc_replay_rate()
        start_time_s = clamp(
            float(self.rc_replay_position_var.get()),
            0.0,
            max(self._rc_replay_duration_s, 0.0),
        )
        if start_time_s >= max(self._rc_replay_duration_s - 0.01, 0.0):
            start_time_s = 0.0
            self._set_rc_replay_position(0.0, force=True)
        samples = list(self._rc_replay_samples)
        self._rc_replay_thread = threading.Thread(
            target=self._run_rc_replay,
            args=(samples, rate, start_time_s),
            daemon=True,
        )
        self._rc_replay_thread.start()

    def _toggle_rc_replay_pause(self) -> None:
        if not self._rc_replay_running():
            return
        if self._rc_replay_pause_event.is_set():
            self._rc_replay_pause_event.clear()
            self.rc_replay_pause_button.config(text="Pause")
            self._set_rc_replay_status("replay: running")
        else:
            self._rc_replay_pause_event.set()
            self.rc_replay_pause_button.config(text="Resume")
            self._set_rc_replay_status("replay: paused")

    def _stop_rc_replay(self) -> None:
        running = self._rc_replay_running()
        self._rc_replay_stop_event.set()
        self._rc_replay_pause_event.clear()
        self.rc_replay_pause_button.config(text="Pause")
        self.node.publish_rc_release()
        self._guided_control_prev = False
        self._rc_override_prev = False
        self._set_rc_replay_status("replay: stopping" if running else "replay: stopped")

    def _run_rc_replay(self, samples: list[RcReplaySample], rate: float, start_time_s: float) -> None:
        local_duration_s = samples[-1].time_s if samples else 0.0
        idx = self._rc_replay_sample_index_for_time(start_time_s)
        start_wall = time.monotonic() - start_time_s / rate
        paused_since: float | None = None
        last_status_wall = 0.0
        last_position_wall = 0.0
        stopped = False

        while idx < len(samples):
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            seek_time_s = self._consume_rc_replay_seek()
            if seek_time_s is not None:
                idx = self._rc_replay_sample_index_for_time(seek_time_s)
                start_wall = time.monotonic() - seek_time_s / rate
                self._set_rc_replay_status(f"replay: seek {format_replay_time(seek_time_s)}")
                continue

            while self._rc_replay_pause_event.is_set() and not self._rc_replay_stop_event.is_set():
                if paused_since is None:
                    paused_since = time.monotonic()
                seek_time_s = self._consume_rc_replay_seek()
                if seek_time_s is not None:
                    idx = self._rc_replay_sample_index_for_time(seek_time_s)
                    start_wall = time.monotonic() - seek_time_s / rate
                    paused_since = time.monotonic()
                    self._set_rc_replay_position(seek_time_s, force=False)
                    self._set_rc_replay_status(f"replay: paused at {format_replay_time(seek_time_s)}")
                time.sleep(0.03)
            if paused_since is not None:
                start_wall += time.monotonic() - paused_since
                paused_since = None
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            sample = samples[idx]
            target_wall = start_wall + sample.time_s / rate
            seek_applied = False
            while not self._rc_replay_stop_event.is_set():
                seek_time_s = self._consume_rc_replay_seek()
                if seek_time_s is not None:
                    idx = self._rc_replay_sample_index_for_time(seek_time_s)
                    start_wall = time.monotonic() - seek_time_s / rate
                    self._set_rc_replay_status(f"replay: seek {format_replay_time(seek_time_s)}")
                    seek_applied = True
                    break
                remaining = target_wall - time.monotonic()
                if remaining <= 0.0:
                    break
                now = time.monotonic()
                if now - last_position_wall > 0.15:
                    elapsed_s = clamp((now - start_wall) * rate, 0.0, local_duration_s)
                    self._set_rc_replay_position(elapsed_s)
                    last_position_wall = now
                time.sleep(min(remaining, 0.02))
            if idx >= len(samples):
                break
            if seek_applied:
                continue
            if self._rc_replay_stop_event.is_set():
                stopped = True
                break

            if not self.node.publish_rc_channels(sample.channels):
                stopped = True
                self._set_rc_replay_status("replay failed: rc override publisher unavailable")
                break

            now = time.monotonic()
            if now - last_position_wall > 0.15:
                self._set_rc_replay_position(sample.time_s)
                last_position_wall = now
            if now - last_status_wall > 0.5:
                last_status_wall = now
                self._set_rc_replay_status(
                    f"replay: {idx + 1}/{len(samples)}  {format_replay_time(sample.time_s)}  rate={rate:g}x"
                )
            idx += 1

        self.node.publish_rc_release()
        self._rc_replay_stop_event.clear()
        self._rc_replay_pause_event.clear()
        self._set_rc_replay_pause_button("Pause")
        if stopped:
            self._set_rc_replay_status("replay: stopped")
        else:
            self._set_rc_replay_position(local_duration_s, force=True)
            self._set_rc_replay_status("replay: finished")
