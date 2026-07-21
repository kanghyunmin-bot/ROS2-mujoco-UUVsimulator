"""Dedicated SITL MAVLink/JSON polling thread for Ros2Bridge."""

from __future__ import annotations

import threading
import time


def start_sitl_poll_thread(self) -> None:
    if not getattr(self, "_sitl_poll_thread_enabled", False):
        return
    if not getattr(self, "enable_sitl", False) or getattr(self, "_sitl_transport", None) is None:
        return
    if self._sitl_poll_thread is not None:
        return
    self._sitl_poll_stop.clear()
    self._sitl_poll_thread = threading.Thread(
        target=self._sitl_poll_loop,
        name="uuv_sitl_poll",
        daemon=True,
    )
    self._sitl_poll_thread.start()
    print(
        "[ros2_bridge] dedicated SITL poll thread enabled "
        f"(hz={self._sitl_poll_thread_hz:.1f})",
        flush=True,
    )


def sitl_poll_loop(self) -> None:
    period_s = 1.0 / max(float(self._sitl_poll_thread_hz), 1.0)
    next_wall = time.monotonic()
    while not self._sitl_poll_stop.is_set():
        now_wall = time.monotonic()
        if now_wall + 1.0e-9 < next_wall:
            time.sleep(min(next_wall - now_wall, period_s))
            continue

        transport = getattr(self, "_sitl_transport", None)
        if transport is None:
            break
        try:
            with self._sitl_transport_lock:
                if self._sitl_transport is None:
                    break
                self._sitl_transport.poll_servo()
        except Exception as exc:
            if not getattr(self, "_sitl_poll_error_reported", False):
                self._sitl_poll_error_reported = True
                print(f"[ros2_bridge] SITL poll thread warning: {exc}", flush=True)

        next_wall += period_s
        now_wall = time.monotonic()
        if now_wall - next_wall > period_s:
            next_wall = now_wall + period_s


def stop_sitl_poll_thread(self) -> None:
    self._sitl_poll_stop.set()
    poll_thread = self._sitl_poll_thread
    if poll_thread is not None and poll_thread.is_alive() and poll_thread is not threading.current_thread():
        try:
            poll_thread.join(timeout=1.0)
        except Exception:
            pass
    self._sitl_poll_thread = None


def sitl_poll_thread_active(self) -> bool:
    poll_thread = getattr(self, "_sitl_poll_thread", None)
    return bool(
        getattr(self, "_sitl_poll_thread_enabled", False)
        and poll_thread is not None
        and poll_thread.is_alive()
    )


__all__ = [
    "sitl_poll_loop",
    "sitl_poll_thread_active",
    "start_sitl_poll_thread",
    "stop_sitl_poll_thread",
]
