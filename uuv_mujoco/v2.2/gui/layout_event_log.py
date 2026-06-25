"""Event log widgets for the GUI telemetry panel."""

from __future__ import annotations

from .config import GROUP_PADDING
from .runtime import tk, ttk


def build_event_log(owner, left) -> None:
    log_box = ttk.LabelFrame(left, text="Events", padding=GROUP_PADDING)
    log_box.grid(row=2, column=0, sticky="nsew")
    owner.event_list = tk.Listbox(
        log_box,
        activestyle="none",
        bg="#ffffff",
        fg="#0f172a",
        selectbackground="#dbeafe",
        selectforeground="#0f172a",
        highlightthickness=1,
        highlightbackground="#cbd5e1",
        borderwidth=0,
        font=("TkDefaultFont", 10),
    )
    owner.event_list.pack(fill=tk.BOTH, expand=True)
