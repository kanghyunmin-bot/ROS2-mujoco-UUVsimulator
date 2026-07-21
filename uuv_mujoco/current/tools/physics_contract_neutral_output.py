"""CSV and summary helpers for neutral open-plant contract sims."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np

from physics_contract_types import NeutralMotionSamples, NeutralSimSummary


NEUTRAL_SIM_CSV_HEADER = [
    "sim_time",
    "base_depth_m",
    "vz_down_mps",
    "buoyancy_n",
    "weight_n",
    "net_up_n",
    "submerged_fraction",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "angular_rate_x_rad_s",
    "angular_rate_y_rad_s",
    "angular_rate_z_rad_s",
    "angular_speed_rad_s",
]


def open_neutral_csv_writer(output_csv: Path) -> tuple[Any, csv.writer]:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    file_obj = output_csv.open("w", newline="")
    writer = csv.writer(file_obj)
    writer.writerow(NEUTRAL_SIM_CSV_HEADER)
    return file_obj, writer


def write_neutral_sample(
    writer: csv.writer,
    *,
    sim_time: float,
    depth: float,
    vz_down: float,
    buoyancy_z: float,
    weight: float,
    submerged_fraction: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    angular_rate_x_rad_s: float,
    angular_rate_y_rad_s: float,
    angular_rate_z_rad_s: float,
    angular_speed_rad_s: float,
) -> None:
    writer.writerow(
        [
            float(sim_time),
            depth,
            vz_down,
            float(buoyancy_z),
            weight,
            float(buoyancy_z - weight),
            submerged_fraction,
            roll_deg,
            pitch_deg,
            yaw_deg,
            angular_rate_x_rad_s,
            angular_rate_y_rad_s,
            angular_rate_z_rad_s,
            angular_speed_rad_s,
        ]
    )


def build_neutral_sim_summary(
    *,
    label: str,
    duration_s: float,
    base_depth_m: float,
    depths: list[float],
    vz_down_values: list[float],
    output_csv: Path,
    motion_samples: NeutralMotionSamples | None = None,
) -> NeutralSimSummary:
    depth_arr = np.asarray(depths, dtype=np.float64)
    vz_arr = np.asarray(vz_down_values, dtype=np.float64)
    start_depth = float(depth_arr[0]) if depth_arr.size else float(base_depth_m)
    end_depth = float(depth_arr[-1]) if depth_arr.size else float(base_depth_m)
    motion = motion_samples if motion_samples is not None else NeutralMotionSamples()
    roll_arr = np.asarray(motion.roll_deg, dtype=np.float64)
    pitch_arr = np.asarray(motion.pitch_deg, dtype=np.float64)
    yaw_arr = np.asarray(motion.yaw_deg, dtype=np.float64)
    wx_arr = np.asarray(motion.angular_rate_x_rad_s, dtype=np.float64)
    wy_arr = np.asarray(motion.angular_rate_y_rad_s, dtype=np.float64)
    wz_arr = np.asarray(motion.angular_rate_z_rad_s, dtype=np.float64)
    angular_speed_arr = np.asarray(motion.angular_speed_rad_s, dtype=np.float64)

    def _start(values: np.ndarray) -> float:
        return float(values[0]) if values.size else 0.0

    def _end(values: np.ndarray) -> float:
        return float(values[-1]) if values.size else 0.0

    def _max_abs(values: np.ndarray) -> float:
        return float(np.max(np.abs(values))) if values.size else 0.0

    return NeutralSimSummary(
        label=label,
        duration_s=float(duration_s),
        start_depth_m=start_depth,
        end_depth_m=end_depth,
        drift_m=float(end_depth - start_depth),
        max_abs_vz_down_mps=float(np.max(np.abs(vz_arr))) if vz_arr.size else 0.0,
        rms_vz_down_mps=float(np.sqrt(np.mean(vz_arr * vz_arr))) if vz_arr.size else 0.0,
        csv=str(output_csv),
        start_roll_deg=_start(roll_arr),
        end_roll_deg=_end(roll_arr),
        start_pitch_deg=_start(pitch_arr),
        end_pitch_deg=_end(pitch_arr),
        start_yaw_deg=_start(yaw_arr),
        end_yaw_deg=_end(yaw_arr),
        max_abs_roll_deg=_max_abs(roll_arr),
        max_abs_pitch_deg=_max_abs(pitch_arr),
        max_abs_yaw_deg=_max_abs(yaw_arr),
        max_abs_angular_rate_x_rad_s=_max_abs(wx_arr),
        max_abs_angular_rate_y_rad_s=_max_abs(wy_arr),
        max_abs_angular_rate_z_rad_s=_max_abs(wz_arr),
        max_angular_speed_rad_s=_max_abs(angular_speed_arr),
        rms_angular_speed_rad_s=(
            float(np.sqrt(np.mean(angular_speed_arr * angular_speed_arr)))
            if angular_speed_arr.size
            else 0.0
        ),
    )


__all__ = [
    "NEUTRAL_SIM_CSV_HEADER",
    "build_neutral_sim_summary",
    "open_neutral_csv_writer",
    "write_neutral_sample",
]
