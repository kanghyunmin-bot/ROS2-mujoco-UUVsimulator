from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
UUV_DIR = REPO_ROOT / "uuv_mujoco" / "v2.2"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(UUV_DIR) not in sys.path:
    sys.path.insert(0, str(UUV_DIR))

from replay_april1_real_commands_in_mujoco import (  # noqa: E402
    CURRENT_SCENE,
    OfflineUuvReplay,
    extract_real_series,
    summarize_replay,
)


DEFAULT_BAGS = [
    Path("real_robot_ros_bag/extracted_2026_04_01/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"),
    Path("real_robot_ros_bag/extracted_2026_04_01/bag_2026-04-01_20-20-30/bag_2026-04-01_20-20-30_0.db3"),
]
DEFAULT_OUT = Path("document/docsource/runs/rosbag/real_bag_2026_04_01_contact_free_axis_sweep")


CANDIDATES: list[dict[str, Any]] = [
    {
        "name": "baseline_current",
        "command_axis_gain": {"surge": 1.25, "sway": 0.55, "yaw": 0.85, "heave": 0.45},
        "yaw_torque_scale": 1.35,
    },
    {
        "name": "balanced_70",
        "command_axis_gain": {"surge": 0.88, "sway": 0.39, "yaw": 0.65, "heave": 0.45},
        "yaw_torque_scale": 1.25,
    },
    {
        "name": "contact_fit_20_08",
        "command_axis_gain": {"surge": 0.46, "sway": 0.26, "yaw": 0.63, "heave": 0.45},
        "yaw_torque_scale": 1.20,
    },
    {
        "name": "surge_yaw_reduced",
        "command_axis_gain": {"surge": 0.65, "sway": 0.42, "yaw": 0.62, "heave": 0.45},
        "yaw_torque_scale": 1.20,
    },
    {
        "name": "sway_preserve_yaw_reduce",
        "command_axis_gain": {"surge": 0.70, "sway": 0.55, "yaw": 0.62, "heave": 0.45},
        "yaw_torque_scale": 1.20,
    },
]


def metric_value(block: dict[str, Any], key: str = "rmse") -> float | None:
    value = block.get(key)
    if isinstance(value, (int, float)) and np.isfinite(float(value)):
        return float(value)
    return None


def weighted_score(metrics: dict[str, Any]) -> float:
    # Units differ. The weights keep yaw-rate comparable to DVL/depth without
    # hiding failures in any one axis.
    pieces: list[tuple[float, float]] = []
    dvl = metrics.get("dvl_velocity_metrics", {})
    gyro = metrics.get("imu_gyro_metrics", {})
    for axis, weight in (("x", 1.0), ("y", 0.8), ("z", 0.5)):
        value = metric_value(dvl.get(axis, {}))
        if value is not None:
            pieces.append((weight, value))
    for axis, weight in (("x", 0.25), ("y", 0.25), ("z", 0.9)):
        value = metric_value(gyro.get(axis, {}))
        if value is not None:
            pieces.append((weight, value))
    depth_value = metric_value(metrics.get("depth_metrics", {}))
    if depth_value is not None:
        pieces.append((0.6, depth_value))
    depth_rate_value = metric_value(metrics.get("depth_rate_metrics", {}))
    if depth_rate_value is not None:
        pieces.append((0.5, depth_rate_value))
    if not pieces:
        return float("inf")
    return float(sum(weight * value for weight, value in pieces) / sum(weight for weight, _ in pieces))


def short_metrics(summary: dict[str, Any]) -> dict[str, Any]:
    cf = summary.get("contact_free_metrics", {})
    dvl = cf.get("dvl_velocity_metrics", {})
    gyro = cf.get("imu_gyro_metrics", {})
    return {
        "valid_fraction": cf.get("valid_fraction"),
        "score": weighted_score(cf),
        "dvl_rmse": {axis: metric_value(dvl.get(axis, {})) for axis in ("x", "y", "z")},
        "dvl_corr": {axis: dvl.get(axis, {}).get("correlation") for axis in ("x", "y", "z")},
        "gyro_rmse": {axis: metric_value(gyro.get(axis, {})) for axis in ("x", "y", "z")},
        "gyro_corr": {axis: gyro.get(axis, {}).get("correlation") for axis in ("x", "y", "z")},
        "depth_rmse": metric_value(cf.get("depth_metrics", {})),
        "depth_rate_rmse": metric_value(cf.get("depth_rate_metrics", {})),
        "contact_fraction_all": summary.get("physics_summary", {}).get("contact_fraction"),
    }


def run_candidate(candidate: dict[str, Any], real, max_duration: float | None, record_dt: float) -> dict[str, Any]:
    profile_overrides = {
        "command_axis_gain": candidate["command_axis_gain"],
        "yaw_torque_scale": candidate["yaw_torque_scale"],
    }
    replay = OfflineUuvReplay(
        scene=CURRENT_SCENE,
        profile_name="current",
        fluid_model="current",
        profile_overrides=profile_overrides,
    )
    sim = replay.run(
        real,
        command_source="rc_override",
        max_duration_s=max_duration,
        record_dt_s=record_dt,
    )
    cfg = {
        "name": candidate["name"],
        "scene": CURRENT_SCENE,
        "profile": "current",
        "fluid_model": "current",
        "command_source": "rc_override",
        "profile_overrides": profile_overrides,
    }
    summary = summarize_replay(real, sim, cfg)
    return {"summary": summary, "short": short_metrics(summary)}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--max-duration", type=float, default=160.0)
    parser.add_argument("--record-dt", type=float, default=0.02)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "script": Path(__file__).name,
        "max_duration_s": args.max_duration,
        "record_dt_s": args.record_dt,
        "candidates": {},
    }
    real_series = [extract_real_series(path) for path in DEFAULT_BAGS]
    for candidate in CANDIDATES:
        candidate_result: dict[str, Any] = {"config": candidate, "bags": {}, "mean_score": None}
        scores = []
        for real in real_series:
            result = run_candidate(candidate, real, args.max_duration, args.record_dt)
            candidate_result["bags"][real.name] = result["short"]
            score = result["short"]["score"]
            if np.isfinite(score):
                scores.append(float(score))
        if scores:
            candidate_result["mean_score"] = float(np.mean(scores))
        payload["candidates"][candidate["name"]] = candidate_result
        print(f"[sweep] {candidate['name']} mean_score={candidate_result['mean_score']}", flush=True)

    ranked = sorted(
        payload["candidates"].values(),
        key=lambda item: float("inf") if item["mean_score"] is None else float(item["mean_score"]),
    )
    payload["ranking"] = [
        {
            "name": item["config"]["name"],
            "mean_score": item["mean_score"],
            "command_axis_gain": item["config"]["command_axis_gain"],
            "yaw_torque_scale": item["config"]["yaw_torque_scale"],
        }
        for item in ranked
    ]
    out_path = args.output_dir / "contact_free_axis_sweep_summary.json"
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"[sweep] wrote {out_path}")


if __name__ == "__main__":
    main()
