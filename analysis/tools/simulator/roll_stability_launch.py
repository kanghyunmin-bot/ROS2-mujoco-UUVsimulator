"""Launch and teardown helpers for roll stability sweeps."""

from __future__ import annotations

import subprocess
from pathlib import Path

from roll_stability_launch_process import interrupt_process_group, launch_headless_candidate, wait_after_interrupt
from roll_stability_launch_readiness import wait_for_readiness_marker


def stop_stack(root: Path, reset_script: Path) -> None:
    subprocess.run(
        [str(reset_script), "--with-qgc-stop"],
        cwd=root,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def start_candidate_launcher(*, root: Path, start_script: Path, launch_log: Path) -> subprocess.Popen:
    return launch_headless_candidate(root=root, start_script=start_script, launch_log=launch_log)


def wait_for_launcher_ready(proc: subprocess.Popen, launch_log: Path, timeout_s: float = 90.0) -> None:
    wait_for_readiness_marker(proc, launch_log, timeout_s)


def terminate_candidate_launcher(proc: subprocess.Popen, *, root: Path, reset_script: Path) -> None:
    interrupt_process_group(proc)
    stop_stack(root, reset_script)
    wait_after_interrupt(proc, timeout_s=5.0)
    stop_stack(root, reset_script)


__all__ = [
    "start_candidate_launcher",
    "stop_stack",
    "terminate_candidate_launcher",
    "wait_for_launcher_ready",
]
