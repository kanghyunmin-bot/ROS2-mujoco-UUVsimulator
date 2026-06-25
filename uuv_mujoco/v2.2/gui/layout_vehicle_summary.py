"""Vehicle summary widgets for the GUI telemetry panel."""

from __future__ import annotations

from .config import GROUP_PADDING
from .runtime import ttk


def build_vehicle_summary(owner, left) -> None:
    summary = ttk.LabelFrame(left, text="Vehicle Summary", padding=GROUP_PADDING)
    summary.grid(row=0, column=0, sticky="ew", pady=(0, 6))
    summary.columnconfigure(0, weight=1)

    summary_header = ttk.Frame(summary)
    summary_header.grid(row=0, column=0, sticky="ew")
    summary_header.columnconfigure(0, weight=1)
    ttk.Label(
        summary_header,
        textvariable=owner.vehicle_summary_var,
        anchor="w",
        font=("TkDefaultFont", 10, "bold"),
    ).grid(row=0, column=0, sticky="ew")
    owner.vehicle_details_button = ttk.Button(
        summary_header,
        text="Details >",
        width=10,
        command=owner._toggle_vehicle_details,
    )
    owner.vehicle_details_button.grid(row=0, column=1, sticky="e", padx=(6, 0))

    ttk.Label(summary, textvariable=owner.motion_summary_var, anchor="w").grid(
        row=1, column=0, sticky="ew", pady=(2, 0)
    )
    ttk.Label(summary, textvariable=owner.control_summary_var, anchor="w").grid(
        row=2, column=0, sticky="ew", pady=(1, 0)
    )
    build_vehicle_detail_rows(owner, summary)


def build_vehicle_detail_rows(owner, summary) -> None:
    owner.vehicle_details_frame = ttk.Frame(summary)
    owner.vehicle_details_frame.grid(row=3, column=0, sticky="ew", pady=(4, 0))
    owner.vehicle_details_frame.columnconfigure(0, weight=1)
    detail_vars = (
        owner.status_var,
        owner.mode_var,
        owner.battery_var,
        owner.pose_var,
        owner.vel_var,
        owner.imu_var,
        owner.autopilot_var,
        owner.depth_target_var,
        owner.depth_source_var,
        owner.age_var,
    )
    for row, var in enumerate(detail_vars):
        ttk.Label(owner.vehicle_details_frame, textvariable=var, anchor="w").grid(
            row=row, column=0, sticky="ew", pady=1
        )
    owner.vehicle_details_frame.grid_remove()
