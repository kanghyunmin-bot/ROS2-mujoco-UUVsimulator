#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_april1_real_bags import read_bag  # noqa: E402


DEPTH_CANDIDATES = (
    "/depth/pose:depth_positive_m",
    "/depth:data",
)
LOCAL_POSE_CANDIDATES = (
    "/mavros/local_position/pose:xyz_m",
    "/mavros/local_position/odom:xyz_m",
)


def resolve_db3(path: Path) -> Path:
    path = Path(path).expanduser()
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3")) or sorted(path.glob("**/*.db3"))
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"No .db3 file found at {path}")


def _window_values(t: np.ndarray, values: np.ndarray, start_s: float, window_s: float) -> np.ndarray:
    t = np.asarray(t, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float).reshape(-1)
    end_s = float(start_s) + float(window_s)
    mask = np.isfinite(t) & np.isfinite(values) & (t >= float(start_s)) & (t <= end_s)
    values = values[mask]
    return values[np.isfinite(values)]


def _reduce(values: np.ndarray, percentile: float | None) -> float:
    if values.size == 0:
        raise ValueError("empty depth window")
    if percentile is None:
        return float(np.mean(values))
    return float(np.percentile(values, float(percentile)))


def estimate_depth(
    db_path: Path,
    start_s: float,
    window_s: float,
    percentile: float | None,
    source: str = "depth",
) -> float:
    bag = read_bag(resolve_db3(db_path))
    last_error: Exception | None = None
    if source == "local_pose":
        for key in LOCAL_POSE_CANDIDATES:
            try:
                t, xyz = bag.array(key, 3)
            except Exception as exc:
                last_error = exc
                continue
            xyz = np.asarray(xyz, dtype=float)
            if xyz.size == 0 or xyz.ndim != 2 or xyz.shape[1] < 3:
                continue
            # MAVROS local_position uses ROS ENU; underwater depth is -z.
            values = _window_values(t, -xyz[:, 2], start_s, window_s)
            if values.size:
                return max(0.0, _reduce(values, percentile))
        if last_error is not None:
            raise RuntimeError(f"Could not read local pose depth from {db_path}: {last_error}") from last_error
        raise RuntimeError(f"No local pose depth samples found in {db_path}")

    for key in DEPTH_CANDIDATES:
        try:
            t, depth = bag.array(key)
        except Exception as exc:
            last_error = exc
            continue
        depth = np.asarray(depth, dtype=float).reshape(-1)
        values = _window_values(t, depth, start_s, window_s)
        if values.size:
            return _reduce(values, percentile)
    if last_error is not None:
        raise RuntimeError(f"Could not read depth from {db_path}: {last_error}") from last_error
    raise RuntimeError(f"No depth samples found in {db_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Estimate initial positive-down depth from a rosbag2 DB.")
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--window-s", type=float, default=5.0)
    parser.add_argument("--percentile", type=float, default=None)
    parser.add_argument("--source", choices=("depth", "local_pose"), default="depth")
    parser.add_argument("--precision", type=int, default=4)
    args = parser.parse_args()
    depth_m = estimate_depth(args.bag, args.start_offset_s, args.window_s, args.percentile, args.source)
    print(f"{depth_m:.{int(args.precision)}f}")


if __name__ == "__main__":
    main()
