#!/usr/bin/env python3
"""Benchmark single-hydrophone drive/pause schedules through the web GUI."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import time
import urllib.request
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
STATUS_PATH = ROOT / "sim/current/generated/mission_fsm_status.json"
OUTPUT_DIR = ROOT / "sim/current/generated"


def request_json(url: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"} if data is not None else {},
        method="POST" if data is not None else "GET",
    )
    with urllib.request.urlopen(request, timeout=10.0) as response:
        return json.load(response)


def command(base_url: str, name: str, **values: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {"command": name}
    payload.update(values)
    return request_json(f"{base_url}/api/command", payload)


def wait_stack(base_url: str, *, running: bool, timeout_s: float = 45.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = request_json(f"{base_url}/api/status")
        processes = status.get("processes", {})
        sim_ready = bool(processes.get("sim_running"))
        mavros_ready = bool(processes.get("mavros_running"))
        if running and sim_ready and mavros_ready:
            return
        if not running and not sim_ready:
            return
        time.sleep(0.5)
    raise TimeoutError(f"sim stack did not reach running={running}")


def read_status() -> dict[str, Any] | None:
    try:
        return json.loads(STATUS_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def parse_variant(spec: str) -> dict[str, float | str]:
    fields = spec.split(":")
    if len(fields) != 3:
        raise ValueError(f"variant must be NAME:DRIVE_S:PAUSE_S, got {spec!r}")
    return {"name": fields[0], "drive_s": float(fields[1]), "pause_s": float(fields[2])}


def log_counts(path: str | None) -> dict[str, int]:
    if not path:
        return {"closest_point_brakes": 0, "range_rejections": 0}
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return {"closest_point_brakes": 0, "range_rejections": 0}
    return {
        "closest_point_brakes": text.count("closest-point reversal detected"),
        "range_rejections": text.count("rejecting stale pinger bearing"),
    }


def run_trial(
    base_url: str,
    variant: dict[str, float | str],
    *,
    timeout_s: float,
    forward: float,
    sway: float,
    sway_period_s: float,
) -> dict[str, Any]:
    command(base_url, "stack_stop")
    wait_stack(base_url, running=False)
    command(base_url, "stack_start")
    wait_stack(base_url, running=True)
    time.sleep(1.0)

    mission_values = {
        "course": "all",
        "own_course": "a",
        "max_targets": 0,
        "rate_hz": 30,
        "transport": "rc_override",
        "dry_run": False,
        "no_pinger": False,
        "nearest_first": True,
        "pinger_forward_fast": forward,
        "pinger_homing_sway_amplitude": sway,
        "pinger_homing_sway_period_s": sway_period_s,
        "pinger_homing_drive_s": variant["drive_s"],
        "pinger_homing_pause_s": variant["pause_s"],
    }
    response = command(base_url, "gt_mission_start", values=mission_values)
    command(base_url, "arm", value=True)
    mission_log = response.get("log")
    wall_started: float | None = None
    final_status: dict[str, Any] = {}
    outcome = "timeout"
    wall_elapsed_s: float | None = None
    startup_deadline = time.monotonic() + 60.0
    next_arm_retry = time.monotonic() + 2.0
    try:
        while True:
            now = time.monotonic()
            status = read_status()
            if status is None:
                if wall_started is None and now >= startup_deadline:
                    outcome = "startup_timeout"
                    break
                time.sleep(0.1)
                continue
            final_status = status
            state = str(status.get("state", ""))
            if state == "WAIT_ARM" and now >= next_arm_retry:
                command(base_url, "arm", value=True)
                next_arm_retry = now + 2.0
            if wall_started is None and state.startswith("PINGER_"):
                wall_started = now
            if wall_started is not None and now - wall_started >= timeout_s:
                outcome = "timeout"
                wall_elapsed_s = now - wall_started
                break
            if wall_started is None and now >= startup_deadline:
                outcome = "startup_timeout"
                break
            if state == "FAILED" or status.get("failure"):
                outcome = "failed"
                wall_elapsed_s = None if wall_started is None else time.monotonic() - wall_started
                break
            if (
                int(status.get("detached_count", 0)) >= 1
                and float(status.get("pinger_completed_elapsed_s", 0.0)) > 0.0
            ):
                outcome = "detached"
                wall_elapsed_s = None if wall_started is None else time.monotonic() - wall_started
                break
            time.sleep(0.1)
    finally:
        command(base_url, "gt_mission_stop")
        time.sleep(0.5)

    result = {
        **variant,
        "outcome": outcome,
        "wall_elapsed_s": wall_elapsed_s,
        "fsm_elapsed_s": final_status.get("pinger_completed_elapsed_s"),
        "path_length_m": final_status.get("pinger_path_length_m"),
        "search_reentries": final_status.get("pinger_search_reentry_count"),
        "recovery_count": final_status.get("pinger_recovery_count"),
        "final_acoustic_range_m": final_status.get("acoustic_range_m"),
        "collector_target_id": final_status.get("collector_target_id"),
        "failure": final_status.get("failure", ""),
        "mission_log": mission_log,
    }
    result.update(log_counts(str(mission_log) if mission_log else None))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8878")
    parser.add_argument("--timeout-s", type=float, default=220.0)
    parser.add_argument("--forward", type=float, default=0.55)
    parser.add_argument("--sway", type=float, default=0.05)
    parser.add_argument("--sway-period-s", type=float, default=6.0)
    parser.add_argument(
        "--variant",
        action="append",
        default=[],
        help="NAME:DRIVE_S:PAUSE_S; repeat for multiple schedules",
    )
    args = parser.parse_args()
    variants = [parse_variant(value) for value in args.variant] or [
        parse_variant("continuous:0:0"),
        parse_variant("drive4_pause0.5:4:0.5"),
        parse_variant("drive2.5_pause0.5:2.5:0.5"),
    ]

    results: list[dict[str, Any]] = []
    for variant in variants:
        print(f"[benchmark] starting {variant['name']}", flush=True)
        result = run_trial(
            args.url,
            variant,
            timeout_s=args.timeout_s,
            forward=args.forward,
            sway=args.sway,
            sway_period_s=args.sway_period_s,
        )
        results.append(result)
        print(json.dumps(result, ensure_ascii=True, sort_keys=True), flush=True)

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output = OUTPUT_DIR / f"pinger_duty_cycle_benchmark_{stamp}.json"
    output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"[benchmark] results={output}", flush=True)
    return 0 if all(item["outcome"] == "detached" for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
