"""Terminal summary output for ALT_HOLD diagnostics."""

from __future__ import annotations

import math
from pathlib import Path

from althold_diagnostics_contract import Snapshot
from althold_diagnostics_series import finite


def print_summary(rows: list[Snapshot], *, csv_path: Path, plot_path: Path, plot_enabled: bool) -> None:
    rc3 = finite([r.rc_in_ch3 for r in rows])
    expected_rc3 = finite([r.manual_expected_rc3 for r in rows])
    expected_climb = finite([r.manual_expected_althold_climb_cm_s for r in rows])
    depth = finite([r.mujoco_depth_m for r in rows])
    rate = finite([r.mujoco_depth_rate_down_mps for r in rows])
    servo_mean = finite([r.rc_out_vertical_mean for r in rows])
    plant_cmd = finite([r.rc_out_vertical_plant_cmd_norm for r in rows])
    mujoco_roll_deg = finite([math.degrees(r.mujoco_roll_rad) for r in rows])
    mujoco_pitch_deg = finite([math.degrees(r.mujoco_pitch_rad) for r in rows])
    sitl_roll_deg = finite([math.degrees(r.sitl_att_roll_rad) for r in rows])
    sitl_pitch_deg = finite([math.degrees(r.sitl_att_pitch_rad) for r in rows])
    sitl_rollspeed = finite([r.sitl_att_rollspeed_rad_s for r in rows])
    sitl_pitchspeed = finite([r.sitl_att_pitchspeed_rad_s for r in rows])
    live_accel_x = finite([r.live_json_accel_x_mps2 for r in rows])
    live_accel_y = finite([r.live_json_accel_y_mps2 for r in rows])
    live_accel_z = finite([r.live_json_accel_z_mps2 for r in rows])
    live_json_roll_deg = finite([math.degrees(r.live_json_roll_rad) for r in rows])
    live_json_pitch_deg = finite([math.degrees(r.live_json_pitch_rad) for r in rows])

    print(f"[althold_diag] wrote CSV: {csv_path}")
    if plot_enabled and plot_path.exists():
        print(f"[althold_diag] wrote plot: {plot_path}")
    _print_range("manual expected RC3 range", expected_rc3, "PWM")
    _print_signed_range("manual expected ALT_HOLD climb range", expected_climb, "cm/s")
    _print_rc_in_range(rc3)
    _print_range("vertical servo mean range", servo_mean, "PWM")
    _print_signed_range("MuJoCo vertical plant command range", plant_cmd, "")
    _print_signed_range("MuJoCo roll range", mujoco_roll_deg, "deg")
    _print_signed_range("MuJoCo pitch range", mujoco_pitch_deg, "deg")
    _print_signed_range("SITL ATT roll range", sitl_roll_deg, "deg")
    _print_signed_range("SITL ATT pitch range", sitl_pitch_deg, "deg")
    _print_signed_range("SITL ATT rollspeed range", sitl_rollspeed, "rad/s")
    _print_signed_range("SITL ATT pitchspeed range", sitl_pitchspeed, "rad/s")
    _print_signed_range("live JSON accel_body x range", live_accel_x, "m/s^2")
    _print_signed_range("live JSON accel_body y range", live_accel_y, "m/s^2")
    _print_signed_range("live JSON accel_body z range", live_accel_z, "m/s^2")
    _print_signed_range("live JSON attitude roll range", live_json_roll_deg, "deg")
    _print_signed_range("live JSON attitude pitch range", live_json_pitch_deg, "deg")
    if len(depth) >= 2:
        print(f"[althold_diag] MuJoCo depth drift: {depth[-1] - depth[0]:+.3f} m")
    if rate:
        rms = math.sqrt(sum(v * v for v in rate) / len(rate))
        print(f"[althold_diag] MuJoCo vertical-rate RMS: {rms:.4f} m/s")


def _print_range(label: str, values: list[float], unit: str) -> None:
    if not values:
        return
    suffix = f" {unit}" if unit else ""
    print(f"[althold_diag] {label}: {min(values):.1f}..{max(values):.1f}{suffix}")


def _print_signed_range(label: str, values: list[float], unit: str) -> None:
    if not values:
        return
    suffix = f" {unit}" if unit else ""
    print(f"[althold_diag] {label}: {min(values):+.1f}..{max(values):+.1f}{suffix}")


def _print_rc_in_range(rc3: list[float]) -> None:
    if not rc3:
        return
    print(f"[althold_diag] /mavros/rc/in ch3 range: {min(rc3):.1f}..{max(rc3):.1f} PWM")
    print("[althold_diag] note: /mavros/rc/in is an RC override mirror in this bridge, not MANUAL_CONTROL's internal ArduSub RC3.")


__all__ = ["print_summary"]
