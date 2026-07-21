"""Vehicle attitude, depth, and RC feedback widgets."""

from __future__ import annotations

from .config import (
    ATTITUDE_CANVAS_HEIGHT,
    ATTITUDE_CANVAS_WIDTH,
    DEPTH_CANVAS_HEIGHT,
    DEPTH_CANVAS_WIDTH,
    GROUP_PADDING,
    RC_VISIBLE_CHANNEL_COUNT,
)
from .runtime import tk, ttk


def build_vehicle_visuals(owner, left) -> None:
    visuals = ttk.Frame(left)
    visuals.grid(row=1, column=0, sticky="nsew", pady=(0, 6))
    visuals.columnconfigure(0, weight=3)
    visuals.columnconfigure(1, weight=2)
    visuals.rowconfigure(0, weight=1)

    attitude_box = ttk.LabelFrame(visuals, text="Attitude", padding=GROUP_PADDING)
    attitude_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
    owner.attitude_canvas = tk.Canvas(
        attitude_box,
        width=ATTITUDE_CANVAS_WIDTH,
        height=ATTITUDE_CANVAS_HEIGHT,
        bg="#0f172a",
        highlightthickness=0,
    )
    owner.attitude_canvas.pack(fill=tk.BOTH, expand=True)

    side_box = ttk.Frame(visuals)
    side_box.grid(row=0, column=1, sticky="nsew")
    side_box.rowconfigure(0, weight=1)
    side_box.rowconfigure(1, weight=1)
    side_box.columnconfigure(0, weight=1)
    build_depth_canvas(owner, side_box)
    build_rc_feedback(owner, side_box)


def build_depth_canvas(owner, side_box) -> None:
    depth_box = ttk.LabelFrame(side_box, text="Depth", padding=GROUP_PADDING)
    depth_box.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
    owner.depth_canvas = tk.Canvas(
        depth_box,
        width=DEPTH_CANVAS_WIDTH,
        height=DEPTH_CANVAS_HEIGHT,
        bg="#081018",
        highlightthickness=0,
    )
    owner.depth_canvas.pack(fill=tk.BOTH, expand=True)


def build_rc_feedback(owner, side_box) -> None:
    rc_box = ttk.LabelFrame(side_box, text="RC Feedback", padding=GROUP_PADDING)
    rc_box.grid(row=1, column=0, sticky="nsew")
    owner._rc_bars = []
    owner._rc_labels = []
    for idx in range(RC_VISIBLE_CHANNEL_COUNT):
        ttk.Label(rc_box, text=f"Ch {idx + 1:02d}").grid(row=idx, column=0, sticky="w")
        bar = ttk.Progressbar(
            rc_box,
            orient=tk.HORIZONTAL,
            maximum=800,
            mode="determinate",
            style="Telemetry.Horizontal.TProgressbar",
        )
        bar.grid(row=idx, column=1, sticky="ew", padx=6)
        value_label = ttk.Label(rc_box, text="0")
        value_label.grid(row=idx, column=2, sticky="e")
        owner._rc_bars.append(bar)
        owner._rc_labels.append(value_label)
    rc_box.columnconfigure(1, weight=1)
