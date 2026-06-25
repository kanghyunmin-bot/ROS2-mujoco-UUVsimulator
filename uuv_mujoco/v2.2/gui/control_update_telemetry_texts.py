"""Telemetry text construction for the GUI refresh loop."""

from __future__ import annotations

import math

from .control_update_format import format_age
from .control_update_telemetry_status import age_text, autopilot_text, status_text
from .control_update_text_models import TelemetryTexts


def build_telemetry_texts(
    *,
    snap,
    backend_label: str,
    rc_mapping_summary: str,
    mode_display: str,
    vehicle_info_supported: bool,
) -> TelemetryTexts:
    px, py, pz = snap.position_xyz
    vx, vy, vz = snap.velocity_xyz
    wx, wy, wz = snap.ang_vel_xyz
    return TelemetryTexts(
        status=status_text(snap, backend_label),
        mode=f"mode: {mode_display}  (raw={snap.mode}, id={snap.mode_id}, state={snap.system_status})",
        vehicle_summary=_vehicle_summary(snap, mode_display),
        battery=_battery_text(snap),
        pose=_pose_text(px, py, pz),
        velocity=_velocity_text(vx, vy, vz, snap.velocity_source),
        imu=(
            f"imu: roll={snap.roll_deg:+.1f}  pitch={snap.pitch_deg:+.1f}  yaw={snap.yaw_deg:+.1f}  "
            f"gyro=({wx:+.2f}, {wy:+.2f}, {wz:+.2f})"
        ),
        motion_summary=_motion_summary(vx, vy, vz, snap),
        autopilot=autopilot_text(snap, rc_mapping_summary, vehicle_info_supported),
        depth_target=f"depth: {snap.depth_m:.2f} m" if math.isfinite(snap.depth_m) else "depth: n/a",
        depth_source=f"depth source: {snap.depth_source}",
        age=age_text(snap),
        ping360_summary=f"{snap.ping360_summary}  age={format_age(snap.ping360_age_s)}",
    )


def _vehicle_summary(snap, mode_display: str) -> str:
    depth_summary = f"{snap.depth_m:.2f} m" if math.isfinite(snap.depth_m) else "n/a"
    vehicle_state = "connected" if snap.connected else "disconnected"
    arm_state = "armed" if snap.armed else "disarmed"
    return f"{vehicle_state} | {arm_state} | {mode_display} | depth {depth_summary}"


def _battery_text(snap) -> str:
    if not math.isfinite(snap.battery_voltage):
        return "battery: n/a"
    batt_pct = snap.battery_percent * 100.0 if math.isfinite(snap.battery_percent) else math.nan
    return f"battery: {snap.battery_voltage:.2f} V, {snap.battery_current:.2f} A, {batt_pct:.0f}%"


def _pose_text(px: float, py: float, pz: float) -> str:
    return "pose: n/a" if not math.isfinite(px) else f"pose: x={px:+.2f}  y={py:+.2f}  z={pz:+.2f}"


def _velocity_text(vx: float, vy: float, vz: float, source: str) -> str:
    if not math.isfinite(vx):
        return f"velocity: n/a  src={source}"
    return f"velocity: x={vx:+.2f}  y={vy:+.2f}  z={vz:+.2f}  src={source}"


def _motion_summary(vx: float, vy: float, vz: float, snap) -> str:
    vel_summary = f"vel ({vx:+.2f}, {vy:+.2f}, {vz:+.2f}) m/s" if math.isfinite(vx) else "vel n/a"
    return f"{vel_summary} | rpy ({snap.roll_deg:+.1f}, {snap.pitch_deg:+.1f}, {snap.yaw_deg:+.1f})"


__all__ = ["build_telemetry_texts"]
