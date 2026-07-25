"""Subprocess helper for physics-profile reset restarts."""

from __future__ import annotations

import subprocess


def execute_physics_reset_process(*, reset_script, sim_stack_dir):
    return subprocess.run(
        [str(reset_script), "--sim-only"],
        cwd=str(sim_stack_dir),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=30,
        start_new_session=True,
    )


__all__ = ["execute_physics_reset_process"]
