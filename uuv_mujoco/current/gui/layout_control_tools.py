"""Right-side auxiliary control panels for replay, tuning, and course layout."""

from __future__ import annotations

from .config import GROUP_PADDING
from .runtime import ttk


def build_control_tools_panel(owner, right):
    tools_column = ttk.LabelFrame(right, text="Tools", padding=GROUP_PADDING)
    owner.control_tools_panel = tools_column
    tools_column.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(0, 6))
    tools_column.columnconfigure(0, weight=1)
    tools_column.rowconfigure(1, weight=1)

    header = ttk.Frame(tools_column)
    header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
    header.columnconfigure(0, weight=1)
    ttk.Label(header, text="Replay / tuning / layout").grid(row=0, column=0, sticky="w")
    owner.control_tools_toggle_button = ttk.Button(
        header,
        text="Hide tools",
        style="Compact.TButton",
        command=owner._toggle_control_tools,
    )
    owner.control_tools_toggle_button.grid(row=0, column=1, sticky="e", padx=(6, 0))

    content = ttk.Frame(tools_column)
    owner.control_tools_content = content
    content.grid(row=1, column=0, sticky="nsew")
    content.columnconfigure(0, weight=1)
    return content


__all__ = ["build_control_tools_panel"]
