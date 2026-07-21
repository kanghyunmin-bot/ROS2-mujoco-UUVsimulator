"""Balance and neutral-simulation console output for physics contract audits."""

from __future__ import annotations

from pathlib import Path

from physics_contract_types import ForceBalance, NeutralSimSummary


def print_balance_section(balances: list[ForceBalance]) -> None:
    print("  balances:")
    for row in balances:
        print(
            f"    {row.label:28s} base={row.base_depth_m:.3f}m "
            f"bar30={row.bar30_depth_m:.3f}m submerged={row.submerged_fraction:.4f} "
            f"net_down={row.net_down_n:+.3f}N accel_down={row.accel_down_mps2:+.4f}m/s^2 "
            f"required_scale={row.required_buoyancy_scale:.6f}"
        )


def print_neutral_sims_section(neutral_sims: list[NeutralSimSummary]) -> None:
    if not neutral_sims:
        return
    print("  neutral open-plant simulations:")
    for row in neutral_sims:
        print(
            f"    {row.label:42s} duration={row.duration_s:.1f}s "
            f"drift={row.drift_m:+.5f}m "
            f"max|vz_down|={row.max_abs_vz_down_mps:.5f}m/s "
            f"rms_vz_down={row.rms_vz_down_mps:.5f}m/s "
            f"rpy_start=({row.start_roll_deg:+.2f},{row.start_pitch_deg:+.2f},{row.start_yaw_deg:+.2f})deg "
            f"rpy_end=({row.end_roll_deg:+.2f},{row.end_pitch_deg:+.2f},{row.end_yaw_deg:+.2f})deg "
            f"max|roll/pitch|=({row.max_abs_roll_deg:.2f},{row.max_abs_pitch_deg:.2f})deg "
            f"max|omega|={row.max_angular_speed_rad_s:.5f}rad/s"
        )


def print_output_paths(csv_path: Path, json_path: Path) -> None:
    print(f"  wrote {csv_path}")
    print(f"  wrote {json_path}")


__all__ = ["print_balance_section", "print_neutral_sims_section", "print_output_paths"]
