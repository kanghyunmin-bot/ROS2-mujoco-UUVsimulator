"""RC replay layout builders."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import tk, ttk


def build_rc_replay_section(owner, parent, *, row: int = 6) -> None:
    replay_box = ttk.LabelFrame(parent, text="RC Override Replay", padding=INNER_PADDING)
    replay_box.grid(row=row, column=0, sticky="ew", pady=(0, 4))
    replay_box.columnconfigure(0, weight=1)

    build_replay_path_row(owner, replay_box)
    build_replay_timeline(owner, replay_box)
    build_replay_controls(owner, replay_box)
    ttk.Label(replay_box, textvariable=owner.rc_replay_status_var, anchor="w", style="Status.TLabel").grid(
        row=3, column=0, sticky="ew", pady=(2, 0)
    )


def build_replay_path_row(owner, replay_box) -> None:
    replay_path_row = ttk.Frame(replay_box)
    replay_path_row.grid(row=0, column=0, sticky="ew")
    replay_path_row.columnconfigure(0, weight=1)
    ttk.Entry(replay_path_row, textvariable=owner.rc_replay_path_var).grid(
        row=0, column=0, sticky="ew", padx=(0, 4)
    )
    ttk.Button(replay_path_row, text="Browse", command=owner._browse_rc_replay_bag).grid(row=0, column=1)


def build_replay_timeline(owner, replay_box) -> None:
    timeline_row = ttk.Frame(replay_box)
    timeline_row.grid(row=1, column=0, sticky="ew", pady=(4, 0))
    timeline_row.columnconfigure(0, weight=1)
    owner.rc_replay_slider = ttk.Scale(
        timeline_row,
        from_=0.0,
        to=1.0,
        orient=tk.HORIZONTAL,
        variable=owner.rc_replay_position_var,
        command=owner._on_rc_replay_slider_changed,
    )
    owner.rc_replay_slider.grid(row=0, column=0, sticky="ew", padx=(0, 6))
    owner.rc_replay_slider.state(["disabled"])
    owner.rc_replay_slider.bind("<Button-1>", owner._on_rc_replay_slider_press)
    owner.rc_replay_slider.bind("<B1-Motion>", owner._on_rc_replay_slider_motion)
    owner.rc_replay_slider.bind("<ButtonRelease-1>", owner._on_rc_replay_slider_release)
    ttk.Label(timeline_row, textvariable=owner.rc_replay_time_var, width=15, anchor="e").grid(
        row=0, column=1, sticky="e"
    )


def build_replay_controls(owner, replay_box) -> None:
    replay_controls = ttk.Frame(replay_box)
    replay_controls.grid(row=2, column=0, sticky="ew", pady=(3, 0))
    ttk.Button(replay_controls, text="Load", command=owner._load_rc_replay).pack(side=tk.LEFT)
    owner.rc_replay_play_button = ttk.Button(replay_controls, text="Play", command=owner._start_rc_replay)
    owner.rc_replay_play_button.pack(side=tk.LEFT, padx=(4, 0))
    owner.rc_replay_pause_button = ttk.Button(
        replay_controls,
        text="Pause",
        command=owner._toggle_rc_replay_pause,
    )
    owner.rc_replay_pause_button.pack(side=tk.LEFT, padx=(4, 0))
    ttk.Button(replay_controls, text="Stop", style="Danger.TButton", command=owner._stop_rc_replay).pack(
        side=tk.LEFT, padx=(4, 0)
    )
    ttk.Label(replay_controls, text="rate").pack(side=tk.LEFT, padx=(10, 2))
    ttk.Entry(replay_controls, width=4, textvariable=owner.rc_replay_rate_var).pack(side=tk.LEFT)
