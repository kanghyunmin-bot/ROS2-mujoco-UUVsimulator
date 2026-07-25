"""Status-line helpers for GUI telemetry text construction."""

from __future__ import annotations

import math

from .control_update_format import format_age


def status_text(snap, backend_label: str) -> str:
    if snap.sitl_mavlink_active and math.isfinite(snap.sitl_mavlink_heartbeat_age_s):
        sitl_mavlink_text = f"sitl_mavlink=ok({snap.sitl_mavlink_heartbeat_age_s:.1f}s)"
    else:
        sitl_mavlink_text = "sitl_mavlink=wait"
    return (
        f"connected={snap.connected}  armed={snap.armed}  guided={snap.guided}  "
        f"manual_input={snap.manual_input}  backend={backend_label}  {sitl_mavlink_text}"
    )


def autopilot_text(snap, rc_mapping_summary: str, vehicle_info_supported: bool) -> str:
    autopilot = snap.autopilot_name
    if not autopilot:
        autopilot = "pending vehicle_info_get" if vehicle_info_supported else "vehicle_info_get unavailable"
    return f"autopilot: {autopilot}  rc-map={rc_mapping_summary}"


def age_text(snap) -> str:
    init_age = format_age(snap.real_start_age_s) if snap.real_start_required else "n/a"
    return (
        "age: "
        f"state={format_age(snap.state_age_s)}, "
        f"imu={format_age(snap.imu_age_s)}, "
        f"pose={format_age(snap.pose_age_s)}, "
        f"depth={format_age(snap.depth_age_s)}, "
        f"rc={format_age(snap.rc_age_s)}, "
        f"init={init_age}"
    )


__all__ = ["age_text", "autopilot_text", "status_text"]
