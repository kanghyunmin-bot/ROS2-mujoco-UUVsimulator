"""Ping360 window lifecycle helpers."""

from __future__ import annotations

from .runtime import tk, ttk


def _toggle_ping360_window(self) -> None:
    if self.ping360_window is not None and self.ping360_window.winfo_exists():
        self._close_ping360_window()
        return
    self._show_ping360_window()


def _show_ping360_window(self) -> None:
    if self.ping360_window is not None and self.ping360_window.winfo_exists():
        self.ping360_window.deiconify()
        self.ping360_window.lift()
        return

    win = tk.Toplevel(self.root)
    win.title("Ping360 Control")
    win.geometry("520x350")
    win.minsize(460, 320)
    win.protocol("WM_DELETE_WINDOW", self._close_ping360_window)
    self.ping360_window = win

    outer = ttk.Frame(win, padding=8)
    outer.pack(fill=tk.BOTH, expand=True)
    outer.columnconfigure(0, weight=1)

    self._build_ping360_status_panel(outer)
    self._build_ping360_power_panel(outer)
    self._build_ping360_view_panel(outer)
    self._build_ping360_params_panel(outer)
    self._build_ping360_footer(outer)


def _close_ping360_window(self) -> None:
    if self.ping360_window is not None and self.ping360_window.winfo_exists():
        self.ping360_window.destroy()
    self.ping360_window = None
