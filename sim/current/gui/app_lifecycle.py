"""Lifecycle methods for the UUV control GUI application."""

from __future__ import annotations

import sys
import traceback

from .app_shutdown_steps import (
    cancel_scheduled_update,
    destroy_root,
    publish_rc_release,
    shutdown_ros_runtime,
    stop_owned_sim_stack,
    stop_rc_replay,
    terminate_child_processes,
)
from .config import RC_FAST_PERIOD_MS, UI_UPDATE_PERIOD_MS


def _gui_log(message: str) -> None:
    print(f"[uuv-gui] {message}", file=sys.stderr, flush=True)


def _spin(self) -> None:
    try:
        self._executor.spin()
    except BaseException:
        traceback.print_exc(file=sys.stderr)
        raise


def _raise_initial_window(self) -> None:
    if self._closed or not self.root.winfo_exists():
        return
    try:
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()
        self.root.attributes("-topmost", True)
        self.root.after(500, lambda: self.root.attributes("-topmost", False))
    except Exception:
        traceback.print_exc(file=sys.stderr)


def _schedule_update(self) -> None:
    if not self._closed and self.root.winfo_exists():
        self._after_id = self.root.after(UI_UPDATE_PERIOD_MS, self._update_ui)


def _schedule_rc_fast_update(self) -> None:
    if not self._closed and self.root.winfo_exists():
        self._rc_fast_after_id = self.root.after(RC_FAST_PERIOD_MS, self._update_rc_fast)


def _update_rc_fast(self) -> None:
    if self._closed or not self.root.winfo_exists():
        return
    try:
        self._publish_active_controls(self._read_control_commands())
    finally:
        self._schedule_rc_fast_update()


def _on_close(self) -> None:
    if self._closed:
        return
    _gui_log("closing")
    self._closed = True
    cancel_scheduled_update(self)
    stop_rc_replay(self)
    self._stop_ping360_view()
    terminate_child_processes(self)
    stop_owned_sim_stack(self)
    publish_rc_release(self)
    shutdown_ros_runtime(self)
    destroy_root(self)


def run(self) -> None:
    _gui_log("entering Tk mainloop")
    self.root.mainloop()
    _gui_log("Tk mainloop exited")
