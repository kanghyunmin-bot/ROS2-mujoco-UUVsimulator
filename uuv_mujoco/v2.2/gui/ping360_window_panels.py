"""Ping360 window panel builders."""

from __future__ import annotations

from .config import INNER_PADDING
from .runtime import tk, ttk


def _build_ping360_status_panel(self, outer: ttk.Frame) -> None:
    status_box = ttk.LabelFrame(outer, text="Status", padding=INNER_PADDING)
    status_box.grid(row=0, column=0, sticky="ew", pady=(0, 6))
    status_box.columnconfigure(0, weight=1)
    ttk.Label(status_box, textvariable=self.ping360_view_status_var, anchor="w").grid(
        row=0, column=0, sticky="ew"
    )
    ttk.Label(status_box, textvariable=self.ping360_summary_var, anchor="w").grid(
        row=1, column=0, sticky="ew", pady=(2, 0)
    )


def _build_ping360_power_panel(self, outer: ttk.Frame) -> None:
    power_box = ttk.LabelFrame(outer, text="Sonar", padding=INNER_PADDING)
    power_box.grid(row=1, column=0, sticky="ew", pady=(0, 6))
    ttk.Checkbutton(
        power_box,
        text="Enabled",
        variable=self.ping360_enabled_var,
        command=self._toggle_ping360_enabled,
    ).pack(side=tk.LEFT)
    ttk.Button(power_box, text="ON", style="Info.TButton", command=lambda: self._set_ping360_enabled(True)).pack(
        side=tk.LEFT, padx=(10, 0)
    )
    ttk.Button(power_box, text="OFF", style="Danger.TButton", command=lambda: self._set_ping360_enabled(False)).pack(
        side=tk.LEFT, padx=(6, 0)
    )


def _build_ping360_view_panel(self, outer: ttk.Frame) -> None:
    view_box = ttk.LabelFrame(outer, text="Viewer", padding=INNER_PADDING)
    view_box.grid(row=2, column=0, sticky="ew", pady=(0, 6))
    ttk.Button(view_box, text="Open RViz/rqt", style="Info.TButton", command=self._start_ping360_view).pack(
        side=tk.LEFT
    )
    ttk.Button(view_box, text="Close Viewer", command=self._stop_ping360_view).pack(
        side=tk.LEFT, padx=(6, 0)
    )


def _build_ping360_params_panel(self, outer: ttk.Frame) -> None:
    params = ttk.LabelFrame(outer, text="Ping360 Params", padding=INNER_PADDING)
    params.grid(row=3, column=0, sticky="ew", pady=(0, 6))
    for col in range(8):
        params.columnconfigure(col, weight=1 if col in (1, 3, 5, 7) else 0)

    ttk.Label(params, text="range").grid(row=0, column=0, sticky="w")
    ttk.Entry(params, width=6, textvariable=self.ping360_range_var).grid(
        row=0, column=1, sticky="ew", padx=(2, 6)
    )
    ttk.Label(params, text="step").grid(row=0, column=2, sticky="w")
    ttk.Entry(params, width=4, textvariable=self.ping360_num_steps_var).grid(
        row=0, column=3, sticky="ew", padx=(2, 6)
    )
    ttk.Label(params, text="gain").grid(row=0, column=4, sticky="w")
    ttk.Entry(params, width=4, textvariable=self.ping360_gain_var).grid(
        row=0, column=5, sticky="ew", padx=(2, 6)
    )
    ttk.Label(params, text="link").grid(row=0, column=6, sticky="w")
    ttk.Combobox(
        params,
        textvariable=self.ping360_interface_var,
        values=("ethernet", "usb", "rs485"),
        state="readonly",
        width=8,
    ).grid(row=0, column=7, sticky="ew", padx=(2, 0))

    ttk.Label(params, text="kHz").grid(row=1, column=0, sticky="w", pady=(4, 0))
    ttk.Entry(params, width=6, textvariable=self.ping360_frequency_var).grid(
        row=1, column=1, sticky="ew", padx=(2, 6), pady=(4, 0)
    )
    ttk.Label(params, text="start").grid(row=1, column=2, sticky="w", pady=(4, 0))
    ttk.Entry(params, width=5, textvariable=self.ping360_start_angle_var).grid(
        row=1, column=3, sticky="ew", padx=(2, 6), pady=(4, 0)
    )
    ttk.Label(params, text="stop").grid(row=1, column=4, sticky="w", pady=(4, 0))
    ttk.Entry(params, width=5, textvariable=self.ping360_stop_angle_var).grid(
        row=1, column=5, sticky="ew", padx=(2, 6), pady=(4, 0)
    )
    ttk.Label(params, text="grad").grid(row=1, column=6, sticky="w", pady=(4, 0))
    ttk.Button(params, text="Apply", style="Info.TButton", command=self._apply_ping360_params).grid(
        row=1, column=7, sticky="ew", padx=(2, 0), pady=(4, 0)
    )


def _build_ping360_footer(self, outer: ttk.Frame) -> None:
    footer = ttk.Frame(outer)
    footer.grid(row=4, column=0, sticky="ew")
    footer.columnconfigure(0, weight=1)
    ttk.Label(
        footer,
        text="Publishes /ping360/config and opens the configured Ping360 view.",
        anchor="w",
        foreground="#64748b",
    ).grid(row=0, column=0, sticky="ew")
    ttk.Button(footer, text="Close", command=self._close_ping360_window).grid(row=0, column=1, padx=(8, 0))
