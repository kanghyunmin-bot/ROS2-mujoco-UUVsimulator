"""Summary writers for roll stability sweeps."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


SUMMARY_KEYS = [
    "candidate",
    "status",
    "score",
    "roll_rms_deg",
    "roll_peak_deg",
    "pitch_rms_deg",
    "gyro_x_rms_rad_s",
    "gyro_y_rms_rad_s",
    "gyro_z_rms_rad_s",
    "yaw_delta_deg",
    "yaw_peak_rate_rad_s",
    "post_gyro_z_rms_rad_s",
    "post_roll_rms_deg",
    "post_pitch_rms_deg",
    "depth_std_m",
    "depth_drift_m",
    "servo_delta_rms_pwm",
    "horizontal_rc_rms_pwm",
    "vertical_rc_rms_pwm",
    "roll_mix_rms_pwm",
    "pitch_mix_rms_pwm",
    "yaw_mix_rms_pwm",
    "rc_valid_samples",
    "stimulus",
    "hold_mode",
    "axis_command",
    "pulse_s_override",
    "wait_ready",
    "note",
    "error",
    "launcher_log",
]


def write_summary(out_dir: Path, results: list[dict[str, Any]]) -> None:
    with (out_dir / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_KEYS)
        writer.writeheader()
        for result in results:
            row = {key: result.get(key, "") for key in SUMMARY_KEYS}
            row["status"] = result.get("status", "ok" if "score" in result else "fail")
            writer.writerow(row)
    (out_dir / "summary.json").write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n")


__all__ = ["SUMMARY_KEYS", "write_summary"]
