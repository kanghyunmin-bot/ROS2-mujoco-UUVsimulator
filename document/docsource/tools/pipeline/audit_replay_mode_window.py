#!/usr/bin/env python3
"""Audit real-bag MAVROS mode coverage for a closed-loop replay window."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _resolve_db3(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3")) or sorted(path.glob("**/*.db3"))
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"No .db3 bag file found at {path}")


def _open_reader(db_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(db_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def audit_window(
    db_path: Path,
    *,
    start_offset_s: float,
    duration_s: float | None,
    expected_mode: str,
) -> dict[str, Any]:
    db_path = _resolve_db3(db_path)
    reader = _open_reader(db_path)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if "/mavros/state" not in topic_types:
        return {
            "status": "missing_state_topic",
            "bag": str(db_path),
            "start_offset_s": float(start_offset_s),
            "duration_s": duration_s,
            "expected_mode": expected_mode,
        }

    state_msg = get_message(topic_types["/mavros/state"])
    t0_ns: int | None = None
    samples: list[tuple[float, str, bool]] = []
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if t0_ns is None:
            t0_ns = int(stamp_ns)
        if topic != "/mavros/state":
            continue
        t_s = (int(stamp_ns) - int(t0_ns)) * 1.0e-9
        if t_s + 1.0e-9 < start_offset_s:
            continue
        if duration_s is not None and t_s > start_offset_s + duration_s + 1.0e-9:
            break
        msg = deserialize_message(raw, state_msg)
        samples.append((float(t_s), str(getattr(msg, "mode", "")), bool(getattr(msg, "armed", False))))

    if not samples:
        return {
            "status": "no_state_samples_in_window",
            "bag": str(db_path),
            "start_offset_s": float(start_offset_s),
            "duration_s": duration_s,
            "expected_mode": expected_mode,
        }

    counts: dict[str, int] = {}
    transitions: list[dict[str, Any]] = []
    prev_mode: str | None = None
    for t_s, mode, armed in samples:
        counts[mode] = counts.get(mode, 0) + 1
        if mode != prev_mode:
            transitions.append({"time_s": t_s, "mode": mode, "armed": armed})
            prev_mode = mode
    majority_mode = max(counts.items(), key=lambda item: item[1])[0]
    expected = str(expected_mode or "").strip()
    matches_expected = not expected or majority_mode == expected
    return {
        "status": "ok" if matches_expected else "mode_mismatch",
        "bag": str(db_path),
        "start_offset_s": float(start_offset_s),
        "duration_s": duration_s,
        "expected_mode": expected,
        "majority_mode": majority_mode,
        "mode_counts": counts,
        "sample_count": len(samples),
        "transitions": transitions,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--expected-mode", default="")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()

    payload = audit_window(
        args.bag,
        start_offset_s=args.start_offset_s,
        duration_s=args.duration_s,
        expected_mode=args.expected_mode,
    )
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(
        "[mode-audit] "
        f"status={payload.get('status')} "
        f"majority={payload.get('majority_mode')} "
        f"expected={payload.get('expected_mode') or 'none'} "
        f"counts={payload.get('mode_counts', {})}",
        flush=True,
    )
    return 2 if args.strict and payload.get("status") == "mode_mismatch" else 0


if __name__ == "__main__":
    raise SystemExit(main())
