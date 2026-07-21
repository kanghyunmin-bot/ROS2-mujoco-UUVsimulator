"""Summary formatting for Ping360 GUI status callbacks."""

from __future__ import annotations

import math

from .node_ping360_status_settings import Ping360StatusSettings


def format_ping360_summary(
    status: Ping360StatusSettings,
    *,
    active_bool: bool | None,
    enabled_bool: bool | None,
) -> str:
    if active_bool is False:
        summary = "ping360: off" if enabled_bool is False else "ping360: inactive"
    elif math.isfinite(status.effective_range):
        summary = (
            f"ping360: req={status.requested_range:.2f}m eff={status.effective_range:.2f}m "
            f"res={status.resolution_cm:.2f}cm step={status.num_steps}/{status.angular_resolution:.1f}deg "
            f"sector={status.start_grad}..{status.stop_grad}grad scan={status.scan_period:.1f}s "
            f"angle={status.angle_deg:.1f}deg"
        )
    else:
        summary = "ping360: waiting for status"
    if isinstance(status.flags, list) and status.flags:
        summary += " flags=" + ",".join(str(flag) for flag in status.flags[:4])
    return summary


__all__ = ["format_ping360_summary"]
