"""Launcher and result IO helpers for roll stability sweeps."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from roll_stability_candidates import Candidate
from roll_stability_launch import (
    start_candidate_launcher,
    stop_stack,
    terminate_candidate_launcher,
    wait_for_launcher_ready,
)
from roll_stability_probe_runtime import collect_probe_metrics
from roll_stability_summary import write_summary


def run_candidate(
    *,
    candidate: Candidate,
    root: Path,
    start_script: Path,
    reset_script: Path,
    out_dir: Path,
    settle_s: float,
    measure_s: float,
    stimulus: str,
    hold_mode: str,
    axis_command: float,
    pulse_s_override: float | None,
    wait_ready: bool,
) -> dict[str, Any]:
    import rclpy

    from roll_stability_probe import StabilityProbe

    cand_dir = out_dir / candidate.name
    cand_dir.mkdir(parents=True, exist_ok=True)
    launch_log = cand_dir / "launcher.log"
    stop_stack(root, reset_script)
    proc = start_candidate_launcher(root=root, start_script=start_script, launch_log=launch_log)

    try:
        if wait_ready:
            wait_for_launcher_ready(proc, launch_log)
        metrics = collect_probe_metrics(
            settle_s=settle_s,
            measure_s=measure_s,
            stimulus=stimulus,
            hold_mode=hold_mode,
            axis_command=axis_command,
            pulse_s_override=pulse_s_override,
        )
    except Exception as exc:
        metrics = {"status": "fail", "error": str(exc)}
    finally:
        terminate_candidate_launcher(proc, root=root, reset_script=reset_script)

    result = {
        "candidate": candidate.name,
        "note": candidate.note,
        "profile_updates": candidate.profile_updates,
        "fluid_angular_scale": candidate.fluid_angular_scale,
        "servo_signs": list(candidate.servo_signs) if candidate.servo_signs is not None else None,
        "launcher_log": str(launch_log),
        "stimulus": stimulus,
        "hold_mode": hold_mode,
        "axis_command": axis_command,
        "pulse_s_override": pulse_s_override,
        "wait_ready": wait_ready,
        **metrics,
    }
    (cand_dir / "metrics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    return result


__all__ = ["run_candidate", "stop_stack", "wait_for_launcher_ready", "write_summary"]
