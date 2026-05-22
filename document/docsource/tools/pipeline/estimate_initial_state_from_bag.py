#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_april1_real_bags import read_bag  # noqa: E402
from estimate_initial_depth_from_bag import resolve_db3  # noqa: E402


RPY_CANDIDATES = (
    "/mavros/imu/data:rpy_rad",
    "/mavros/local_position/pose:rpy_rad",
    "/mavros/local_position/odom:rpy_rad",
)
BODY_VEL_CANDIDATES = (
    "/dvl/twist:linear_m_s",
    "/dvl/odometry:linear_m_s",
)
LOCAL_VEL_CANDIDATES = (
    "/mavros/local_position/velocity_local:linear_m_s",
    "/mavros/local_position/odom:linear_m_s",
)


def _window_matrix(t: np.ndarray, values: np.ndarray, start_s: float, window_s: float, dims: int = 3) -> np.ndarray:
    t = np.asarray(t, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return np.empty((0, dims), dtype=float)
    values = values.reshape((-1, dims))
    n = min(t.size, values.shape[0])
    t = t[:n]
    values = values[:n]
    end_s = float(start_s) + float(window_s)
    mask = np.isfinite(t) & (t >= float(start_s)) & (t <= end_s)
    values = values[mask]
    if values.size == 0:
        return np.empty((0, dims), dtype=float)
    finite = np.all(np.isfinite(values), axis=1)
    return values[finite]


def _median_rpy(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        raise ValueError("empty RPY window")
    roll = float(np.median(values[:, 0]))
    pitch = float(np.median(values[:, 1]))
    yaw = float(math.atan2(np.mean(np.sin(values[:, 2])), np.mean(np.cos(values[:, 2]))))
    return np.array([roll, pitch, yaw], dtype=float)


def _median_vector(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        raise ValueError("empty vector window")
    return np.median(values, axis=0).astype(float)


def _first_available_vector(bag, keys: tuple[str, ...], start_s: float, window_s: float, *, circular_rpy: bool = False) -> np.ndarray:
    last_error: Exception | None = None
    for key in keys:
        try:
            t, values = bag.array(key, 3)
        except Exception as exc:
            last_error = exc
            continue
        window = _window_matrix(t, values, start_s, window_s, 3)
        if window.size:
            return _median_rpy(window) if circular_rpy else _median_vector(window)
    if last_error is not None:
        raise RuntimeError(f"Could not read any of {keys}: {last_error}") from last_error
    raise RuntimeError(f"No samples found for any of {keys}")


def estimate_state(db_path: Path, start_s: float, window_s: float) -> dict[str, list[float]]:
    bag = read_bag(resolve_db3(db_path))
    result: dict[str, list[float]] = {}
    result["rpy_rad"] = _first_available_vector(
        bag,
        RPY_CANDIDATES,
        start_s,
        window_s,
        circular_rpy=True,
    ).tolist()
    try:
        result["body_linear_velocity_m_s"] = _first_available_vector(
            bag,
            BODY_VEL_CANDIDATES,
            start_s,
            window_s,
        ).tolist()
    except RuntimeError:
        result["local_linear_velocity_m_s"] = _first_available_vector(
            bag,
            LOCAL_VEL_CANDIDATES,
            start_s,
            window_s,
        ).tolist()
    return result


def _fmt(values: list[float], precision: int) -> str:
    return " ".join(f"{float(v):.{int(precision)}f}" for v in values)


def main() -> None:
    parser = argparse.ArgumentParser(description="Estimate mid-run initial attitude/velocity from a rosbag2 DB.")
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--window-s", type=float, default=1.0)
    parser.add_argument(
        "--field",
        choices=("json", "rpy", "body-velocity", "local-velocity"),
        default="json",
    )
    parser.add_argument("--precision", type=int, default=6)
    args = parser.parse_args()

    state = estimate_state(args.bag, args.start_offset_s, args.window_s)
    if args.field == "json":
        print(json.dumps(state, indent=2, ensure_ascii=False))
    elif args.field == "rpy":
        print(_fmt(state["rpy_rad"], args.precision))
    elif args.field == "body-velocity":
        if "body_linear_velocity_m_s" not in state:
            raise RuntimeError("body velocity unavailable")
        print(_fmt(state["body_linear_velocity_m_s"], args.precision))
    elif args.field == "local-velocity":
        if "local_linear_velocity_m_s" not in state:
            raise RuntimeError("local velocity unavailable")
        print(_fmt(state["local_linear_velocity_m_s"], args.precision))


if __name__ == "__main__":
    main()
