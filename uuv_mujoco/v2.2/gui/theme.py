"""Visual theme for the MuJoCo UUV control GUI."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


APP_BG = "#e8edf3"
SURFACE = "#f8fafc"
SURFACE_ALT = "#eef4f8"
TEXT = "#0f172a"
MUTED = "#475569"
BORDER = "#cbd5e1"
ACCENT = "#2563eb"
ACCENT_ACTIVE = "#1d4ed8"
INFO = "#0891b2"
INFO_ACTIVE = "#0e7490"
SUCCESS = "#15803d"
SUCCESS_ACTIVE = "#166534"
DANGER = "#dc2626"
DANGER_ACTIVE = "#b91c1c"


def apply_theme(root: tk.Tk, style: ttk.Style) -> None:
    """Apply a compact operations-focused ttk theme."""
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    root.configure(bg=APP_BG)
    style.configure(".", font=("TkDefaultFont", 10), foreground=TEXT)
    style.configure("App.TFrame", background=APP_BG)
    style.configure("TFrame", background=SURFACE)
    style.configure("Header.TFrame", background="#111827")
    style.configure("TLabel", background=SURFACE, foreground=TEXT)
    style.configure("Title.TLabel", background="#111827", foreground="#f8fafc", font=("TkDefaultFont", 15, "bold"))
    style.configure("Subtitle.TLabel", background="#111827", foreground="#cbd5e1", font=("TkDefaultFont", 9))
    style.configure("StatusPill.TLabel", background="#1f2937", foreground="#e5e7eb", padding=(8, 3))
    style.configure("Status.TLabel", background=SURFACE_ALT, foreground=MUTED, padding=(7, 3))
    style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED)

    style.configure("TButton", padding=(8, 4), background="#f1f5f9", foreground=TEXT, bordercolor=BORDER)
    style.map("TButton", background=[("active", "#e2e8f0"), ("pressed", "#cbd5e1")])
    style.configure("Accent.TButton", padding=(8, 4), foreground="white", background=ACCENT, bordercolor=ACCENT)
    style.map("Accent.TButton", background=[("active", ACCENT_ACTIVE), ("pressed", "#1e40af")])
    style.configure("Info.TButton", padding=(8, 4), foreground="white", background=INFO, bordercolor=INFO)
    style.map("Info.TButton", background=[("active", INFO_ACTIVE), ("pressed", "#155e75")])
    style.configure("Success.TButton", padding=(8, 4), foreground="white", background=SUCCESS, bordercolor=SUCCESS)
    style.map("Success.TButton", background=[("active", SUCCESS_ACTIVE), ("pressed", "#14532d")])
    style.configure("Danger.TButton", padding=(8, 4), foreground="white", background=DANGER, bordercolor=DANGER)
    style.map("Danger.TButton", background=[("active", DANGER_ACTIVE), ("pressed", "#991b1b")])

    style.configure("TCheckbutton", padding=(3, 2), background=SURFACE, foreground=TEXT)
    style.configure("TEntry", fieldbackground="#ffffff", bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
    style.configure("TScale", background=SURFACE)
    style.configure("TLabelframe", padding=4, background=SURFACE, bordercolor=BORDER, relief=tk.GROOVE)
    style.configure("TLabelframe.Label", background=SURFACE, foreground=TEXT, font=("TkDefaultFont", 10, "bold"))
    style.configure("Danger.TLabelframe", padding=4, background=SURFACE, bordercolor=DANGER, relief=tk.GROOVE)
    style.configure("Danger.TLabelframe.Label", background=SURFACE, foreground=DANGER, font=("TkDefaultFont", 10, "bold"))
    style.configure(
        "Telemetry.Horizontal.TProgressbar",
        background=INFO,
        troughcolor="#e2e8f0",
        bordercolor=BORDER,
        lightcolor=INFO,
        darkcolor=INFO,
    )
    style.configure("Treeview", background="#ffffff", fieldbackground="#ffffff", foreground=TEXT, rowheight=22)
    style.configure("Treeview.Heading", background=SURFACE_ALT, foreground=TEXT, font=("TkDefaultFont", 9, "bold"))
