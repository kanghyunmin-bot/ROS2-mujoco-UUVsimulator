"""Prepare DFKI BlueROV2 v3 trial streams for a timestamp-preserving replay.

Input files are extracted from the public Zenodo 17360027 v3 archive. Motion
capture uses its local world frame and the onboard attitude uses NED/FRD.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


SOURCE_URL = "https://zenodo.org/records/17360027"
SOURCE_ARCHIVE_MD5 = "f357044a1d6d1f76164c8c94048ef2ae"
PRESSURE_ATMOSPHERIC_PA = 103520.0
ASSUMED_WATER_DENSITY_KG_M3 = 1025.0
GRAVITY_M_S2 = 9.80665


def read_csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, ndmin=1)
    if len(data) < 2 or not np.isfinite(data["time"]).all() or np.any(np.diff(data["time"]) <= 0):
        raise ValueError(f"Invalid timestamps in {path}")
    return data


def interpolate(data: np.ndarray, names: list[str], times: np.ndarray, max_gap: float) -> tuple[np.ndarray, np.ndarray]:
    source_t = data["time"]
    values = np.column_stack([data[name] for name in names])
    finite = np.isfinite(values).all(axis=1)
    source_t, values = source_t[finite], values[finite]
    if len(source_t) < 2:
        raise ValueError(f"Not enough finite samples for {names}")
    right = np.searchsorted(source_t, times, side="right")
    left = np.clip(right - 1, 0, len(source_t) - 2)
    right = left + 1
    valid = (times >= source_t[0]) & (times <= source_t[-1]) & (source_t[right] - source_t[left] <= max_gap)
    result = np.column_stack([np.interp(times, source_t, values[:, i]) for i in range(len(names))])
    # A measurement at a gap edge is valid even if its next neighbour is far away.
    exact = np.clip(np.searchsorted(source_t, times), 0, len(source_t) - 1)
    valid |= np.abs(times - source_t[exact]) <= 1e-9
    result[~valid] = np.nan
    return result, valid


def save(path: Path, values: np.ndarray, names: list[str]) -> None:
    np.savetxt(path, values, delimiter=",", header=",".join(names), comments="", fmt="%.9f")


def prepare(source: Path, output: Path, trial: str) -> dict:
    filenames = ["attitude", "battery", "pose", "pressure2", "qtm_rov", "rc-out", "velocity"]
    streams = {name: read_csv(source / f"{name}.csv") for name in filenames}
    velocity, pose, attitude = (streams[name] for name in ("velocity", "pose", "attitude"))
    marker, pwm, battery, pressure = (streams[name] for name in ("qtm_rov", "rc-out", "battery", "pressure2"))
    start = max(stream["time"][0] for stream in streams.values())
    end = min(stream["time"][-1] for stream in streams.values())
    velocity = velocity[(velocity["time"] >= start) & (velocity["time"] <= end)]
    marker_fields = [f"rov_{name}_{axis}" for name in ("front_left", "front_right", "back_right") for axis in "xyz"]
    # The released v3 attitude is already yaw-aligned by its authors. Unwrap
    # before interpolation so crossing +/-pi cannot create artificial turns.
    unwrapped = attitude.copy()
    for name in ("roll", "pitch", "yaw"):
        unwrapped[name] = np.unwrap(unwrapped[name])

    def sample(times: np.ndarray) -> tuple:
        markers, marker_valid = interpolate(marker, marker_fields, times, 0.04001)
        left, front, rear = markers[:, :3], markers[:, 3:6], markers[:, 6:9]
        forward = front - rear
        forward /= np.linalg.norm(forward, axis=1)[:, None]
        right = front - left
        right -= np.sum(right * forward, axis=1)[:, None] * forward
        right /= np.linalg.norm(right, axis=1)[:, None]
        down = np.cross(forward, right)
        world_v, velocity_valid = interpolate(velocity, [f"linear_{axis}" for axis in "xyz"], times, 0.04)
        body_v = np.column_stack([np.sum(world_v * direction, axis=1) for direction in (forward, right, down)])
        body_valid = marker_valid & velocity_valid & np.isfinite(body_v).all(axis=1)
        body_v[~body_valid] = np.nan
        position, pose_valid = interpolate(pose, [f"pos_{axis}" for axis in "xyz"], times, 0.04)
        attitude_values, attitude_valid = interpolate(
            unwrapped, ["roll", "pitch", "yaw", "roll_speed", "pitch_speed", "yaw_speed"], times, 0.25
        )
        pvalues, pressure_valid = interpolate(pressure, ["press_abs"], times, 0.25)
        vvalues, battery_valid = interpolate(battery, ["voltages_01"], times, 0.25)
        battery_valid &= (vvalues[:, 0] >= 10) & (vvalues[:, 0] <= 20)
        return (body_v, position, attitude_values, pvalues, vvalues, body_valid, marker_valid,
                pose_valid, attitude_valid, pressure_valid, battery_valid)

    original_times = velocity["time"]
    original = sample(original_times)
    initial_candidates = original[5] & original[7] & original[8] & original[9] & original[10]
    candidates = np.flatnonzero(initial_candidates)
    if not len(candidates):
        raise ValueError("No valid common initial state")
    start = float(original_times[candidates[0]])
    end = float(original_times[-1])
    # Keep the original velocity and attitude samples and fill their gaps with
    # a 100 Hz grid. Each channel independently enforces its own gap limit.
    dense = start + np.arange(int(np.floor((end - start) / 0.01)) + 1) * 0.01
    attitude_times = attitude["time"][(attitude["time"] >= start) & (attitude["time"] <= end)]
    times = np.unique(np.round(np.r_[original_times[original_times >= start], attitude_times, dense], 9))
    (body_v, position, attitude_values, pvalues, vvalues, body_valid, marker_valid,
     pose_valid, attitude_valid, pressure_valid, battery_valid) = sample(times)
    pressure_initial = float(pvalues[0, 0])
    depth_initial = (pressure_initial - PRESSURE_ATMOSPHERIC_PA) / (ASSUMED_WATER_DENSITY_KG_M3 * GRAVITY_M_S2)
    initial_body_velocity = body_v[0].tolist()
    initial_rate = attitude_values[0, 3:].tolist()
    rows = np.column_stack([times, body_v[:, :2], -position[:, 2], np.rad2deg(attitude_values[:, :3]), body_v[:, 2], attitude_values[:, 3:]])
    names = ["t_s", "surge_mps", "sway_mps", "depth_m", "roll_deg", "pitch_deg", "yaw_deg", "heave_mps", "roll_rate_radps", "pitch_rate_radps", "yaw_rate_radps"]
    output.mkdir(parents=True, exist_ok=True)
    save(output / "real.csv", rows, names)
    save(output / "validity.csv", np.column_stack([times, body_valid, marker_valid, pose_valid, attitude_valid, pressure_valid, battery_valid]),
         ["t_s", "body_velocity_valid", "marker_valid", "pose_valid", "attitude_valid", "pressure_valid", "battery_valid"])
    # Preserve every received command and its original timestamp, including pre-roll.
    pwm_rows = np.column_stack([pwm["time"]] + [pwm[f"channel{i}"] for i in range(1, 7)])
    save(output / "pwm.csv", pwm_rows, ["t_s"] + [f"pwm{i}" for i in range(1, 7)])
    # Voltage trace is relative to replay start; the first sample is interpolated
    # at that instant. Reject missing interior data rather than silently bridging it.
    bt = battery["time"]
    bv = battery["voltages_01"]
    select = (bt > start) & (bt <= times[-1])
    voltage_t = np.r_[0.0, bt[select] - start]
    voltage = np.r_[vvalues[0, 0], bv[select]]
    if not np.isfinite(voltage).all() or np.any((voltage < 10) | (voltage > 20)) or np.any(np.diff(voltage_t) > 0.25):
        raise ValueError("Battery trace has missing or unsupported samples")
    save(output / "voltage.csv", np.column_stack([voltage_t, voltage]), ["sim_time", "voltage_v"])
    metadata = {
        "source": SOURCE_URL, "source_archive_md5": SOURCE_ARCHIVE_MD5, "trial": trial,
        "source_files_sha256": {name: hashlib.sha256((source / f"{name}.csv").read_bytes()).hexdigest() for name in filenames},
        "start_s": start, "end_s": float(times[-1]), "initial_pressure_depth_m": depth_initial,
        "initial_pressure_pa": pressure_initial, "atmospheric_pressure_pa": PRESSURE_ATMOSPHERIC_PA,
        "assumed_water_density_kg_m3": ASSUMED_WATER_DENSITY_KG_M3,
        "initial_body_linear_velocity_frd_mps": initial_body_velocity,
        "initial_body_angular_rate_frd_radps": initial_rate,
        "depth_reference": "negative mocap world z; relative depth should use the first sample",
        "attitude_reference": "published onboard NED roll pitch yaw, yaw unwrapped before interpolation",
        "coverage": {"samples": len(times), "body_velocity_valid": int(body_valid.sum()), "pose_valid": int(pose_valid.sum()), "attitude_valid": int(attitude_valid.sum())},
        "gap_limits_s": {"marker": 0.04001, "velocity": 0.04, "pose": 0.04, "attitude": 0.25, "pressure": 0.25, "battery": 0.25},
    }
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Directory of raw trial CSV files from the v3 archive")
    parser.add_argument("output", type=Path)
    parser.add_argument("--trial", choices=("random", "circle", "serpentine", *(f"{n}m" for n in range(5, 16))),
                        required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output, args.trial), indent=2))


if __name__ == "__main__":
    main()
