"""Compare BlueROV2 replays against the DFKI measured track on a shared 10 Hz grid."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from replay_bluerov2_dfki import read_rows
from rosbag_trend_math import interpolate_valid

METRICS = (
    ("Surge", "surge_mps", "m/s", 1.0),
    ("Sway", "sway_mps", "m/s", 1.0),
    ("Heave", "depth_m", "cm", 100.0),
    ("Roll", "roll_deg", "deg", 1.0),
    ("Pitch", "pitch_deg", "deg", 1.0),
    ("Yaw", "yaw_deg", "deg", 1.0),
)
KINEMATIC_METRICS = (
    ("Surge velocity", "surge_mps", "m/s", 1.0),
    ("Sway velocity", "sway_mps", "m/s", 1.0),
    ("Heave velocity", "heave_mps", "m/s", 1.0),
    ("Roll rate", "roll_rate_radps", "deg/s", 180.0 / np.pi),
    ("Pitch rate", "pitch_rate_radps", "deg/s", 180.0 / np.pi),
    ("Yaw rate", "yaw_rate_radps", "deg/s", 180.0 / np.pi),
)


def yaw_diagnostics(time: np.ndarray, traces: dict, names: list[str]) -> dict:
    """Separate short-window heading response from full-run drift."""
    measured = traces["yaw_measured"]
    valid = traces["yaw_valid"]
    errors = {name: traces[f"yaw_{name}"] - measured for name in names}
    result = {}
    for seconds in (1, 5, 10):
        steps = round(seconds / float(time[1] - time[0]))
        common = valid[steps:] & valid[:-steps]
        if not common.any():
            continue
        result[f"increment_{seconds}s"] = {
            "samples": int(common.sum()),
            "rmse_deg": {
                name: float(np.sqrt(np.mean((error[steps:] - error[:-steps])[common] ** 2)))
                for name, error in errors.items()
            },
        }
    final = np.flatnonzero(valid)[-1]
    result["final_error_deg"] = {name: float(error[final]) for name, error in errors.items()}
    return result


def evaluate(real_path: Path, runs: list[Path]) -> tuple[dict, dict]:
    measured = read_rows(real_path)
    tracks = [measured, *(read_rows(run / "simulation.csv") for run in runs)]
    start = max(track["t_s"][0] for track in tracks)
    end = min(track["t_s"][-1] for track in tracks)
    time = start + np.arange(int(np.floor((end - start) * 10 + 1e-8)) + 1) * .1
    if len(time) < 10:
        raise ValueError("Less than one second of common time support")
    output = {"interval_s": [float(time[0]), float(time[-1])], "metrics": [], "kinematic_metrics": []}
    traces = {"t_s": time}
    for axis, key, unit, scale in METRICS:
        sampled = []
        for track in tracks:
            values = track[key].copy()
            if key == "yaw_deg":
                valid = np.isfinite(values)
                values[valid] = np.rad2deg(np.unwrap(np.deg2rad(values[valid])))
            values = interpolate_valid(track["t_s"], values, time, max_gap_s=.04) * scale
            if key in {"yaw_deg", "depth_m"}:
                if not np.isfinite(values[0]):
                    raise ValueError(f"No common starting value for {key}")
                values -= values[0]
            sampled.append(values)
        common = np.isfinite(sampled).all(axis=0)
        if common.sum() < 10:
            raise ValueError(f"Too few valid paired samples for {axis}")
        scores = {}
        for run, values in zip(runs, sampled[1:], strict=True):
            scores[run.name] = float(np.sqrt(np.mean((values[common] - sampled[0][common]) ** 2)))
            traces[f"{axis.lower()}_{run.name}"] = values
        traces[f"{axis.lower()}_measured"] = sampled[0]
        traces[f"{axis.lower()}_valid"] = common
        output["metrics"].append({"axis": axis, "unit": unit, "samples": int(common.sum()), "rmse": scores})
        if key == "yaw_deg":
            errors = [(values[common] - sampled[0][common] + 180) % 360 - 180 for values in sampled[1:]]
            output["wrapped_yaw_rmse_deg"] = {
                run.name: float(np.sqrt(np.mean(error**2))) for run, error in zip(runs, errors, strict=True)
            }
    for axis, key, unit, scale in KINEMATIC_METRICS:
        sampled = [interpolate_valid(track["t_s"], track[key], time, max_gap_s=.04) * scale for track in tracks]
        common = np.isfinite(sampled).all(axis=0)
        if common.sum() < 10:
            raise ValueError(f"Too few valid paired samples for {axis}")
        scores = {
            run.name: float(np.sqrt(np.mean((values[common] - sampled[0][common]) ** 2)))
            for run, values in zip(runs, sampled[1:], strict=True)
        }
        output["kinematic_metrics"].append({"axis": axis, "unit": unit, "samples": int(common.sum()), "rmse": scores})
        if key == "yaw_rate_radps":
            output["yaw_rate_bias_deg_s"] = {
                run.name: float(np.mean(values[common] - sampled[0][common]))
                for run, values in zip(runs, sampled[1:], strict=True)
            }
            traces["yaw_rate_measured"] = sampled[0]
            for run, values in zip(runs, sampled[1:], strict=True):
                traces[f"yaw_rate_{run.name}"] = values
    output["yaw_diagnostics"] = yaw_diagnostics(time, traces, [run.name for run in runs])
    output["runs"] = {run.name: json.loads((run / "run.json").read_text()) for run in runs}
    return output, traces


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args()
    if len({run.name for run in args.runs}) != len(args.runs):
        parser.error("Run directory names must be unique")
    scores, traces = evaluate(args.real, args.runs)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "rmse.json").write_text(json.dumps(scores, indent=2) + "\n")
    np.savetxt(args.output / "aligned.csv", np.column_stack(list(traces.values())),
               delimiter=",", header=",".join(traces), comments="", fmt="%.9f")
    print(json.dumps(scores["metrics"], indent=2))
