"""CSV and summary helpers for neutral open-plant contract sims."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np

from physics_contract_types import NeutralSimSummary


NEUTRAL_SIM_CSV_HEADER = [
    "sim_time",
    "base_depth_m",
    "vz_down_mps",
    "buoyancy_n",
    "weight_n",
    "net_up_n",
    "submerged_fraction",
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
) -> NeutralSimSummary:
    depth_arr = np.asarray(depths, dtype=np.float64)
    vz_arr = np.asarray(vz_down_values, dtype=np.float64)
    start_depth = float(depth_arr[0]) if depth_arr.size else float(base_depth_m)
    end_depth = float(depth_arr[-1]) if depth_arr.size else float(base_depth_m)
    return NeutralSimSummary(
        label=label,
        duration_s=float(duration_s),
        start_depth_m=start_depth,
        end_depth_m=end_depth,
        drift_m=float(end_depth - start_depth),
        max_abs_vz_down_mps=float(np.max(np.abs(vz_arr))) if vz_arr.size else 0.0,
        rms_vz_down_mps=float(np.sqrt(np.mean(vz_arr * vz_arr))) if vz_arr.size else 0.0,
        csv=str(output_csv),
    )


__all__ = [
    "NEUTRAL_SIM_CSV_HEADER",
    "build_neutral_sim_summary",
    "open_neutral_csv_writer",
    "write_neutral_sample",
]
