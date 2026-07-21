"""Physics parameter editor window construction."""

from __future__ import annotations

import threading

from .physics_window_rows import build_physics_footer, build_physics_grid
from .physics_window_shell import create_physics_window_shell, restore_existing_physics_window


def _show_physics_window(self) -> None:
    if restore_existing_physics_window(self):
        return

    self._load_physics_params_into_fields(silent=True)
    outer, grid = create_physics_window_shell(self)
    build_physics_grid(self, grid)
    build_physics_footer(self, outer)


def _close_physics_window(self) -> None:
    if self.physics_window is not None and self.physics_window.winfo_exists():
        self.physics_window.destroy()
    self.physics_window = None
    self.physics_canvas = None
    self.physics_scroll_frame = None


def _set_physics_status(self, text: str) -> None:
    if threading.current_thread() is threading.main_thread():
        self.physics_status_var.set(text)
        return
    try:
        self.root.after(0, lambda: self.physics_status_var.set(text))
    except Exception:
        pass
