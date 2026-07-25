"""Top-level Tk shell layout for the GUI."""

from __future__ import annotations

from .config import OUTER_PADDING
from .runtime import tk, ttk
from .theme import apply_theme


def build_layout_shell(owner):
    style = ttk.Style(owner.root)
    apply_theme(owner.root, style)

    container = ttk.Frame(owner.root, padding=OUTER_PADDING, style="App.TFrame")
    owner.main_container = container
    container.pack(fill=tk.BOTH, expand=True)
    container.columnconfigure(0, weight=2)
    container.columnconfigure(1, weight=3)
    container.rowconfigure(1, weight=1)

    build_header(owner, container)

    left = ttk.Frame(container)
    owner.telemetry_panel = left
    left.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
    left.columnconfigure(0, weight=1)
    left.rowconfigure(1, weight=1)
    left.rowconfigure(2, weight=1)

    right = ttk.Frame(container)
    owner.control_panel = right
    right.grid(row=1, column=1, sticky="nsew")
    right.columnconfigure(0, weight=3)
    right.columnconfigure(1, weight=2)
    right.rowconfigure(0, weight=1)
    return left, right


def build_header(owner, container) -> None:
    header = ttk.Frame(container, style="Header.TFrame", padding=(12, 9))
    header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
    header.columnconfigure(0, weight=1)

    title_stack = ttk.Frame(header, style="Header.TFrame")
    title_stack.grid(row=0, column=0, sticky="w")
    ttk.Label(title_stack, text="UUV Control GUI", style="Title.TLabel").grid(row=0, column=0, sticky="w")
    ttk.Label(title_stack, text="MuJoCo, ROS2, MAVROS, RViz, Ping360", style="Subtitle.TLabel").grid(
        row=1, column=0, sticky="w", pady=(1, 0)
    )

    status_strip = ttk.Frame(header, style="Header.TFrame")
    status_strip.grid(row=0, column=1, sticky="e")
    for idx, var in enumerate((owner.sim_stack_status_var, owner.ros_pkg_status_var, owner.ping360_summary_var)):
        ttk.Label(status_strip, textvariable=var, style="StatusPill.TLabel").grid(
            row=0, column=idx, sticky="e", padx=(6 if idx else 0, 0)
        )
