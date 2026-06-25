"""Launch-target helpers for GUI-started simulator stacks."""

from __future__ import annotations

import os
from dataclasses import dataclass

from .config import START_DOCKER_SIM_STACK_SCRIPT, START_SIM_STACK_SCRIPT


@dataclass(frozen=True)
class SimStackLaunchTarget:
    backend: str
    start_script: object
    start_label: str


def resolve_sim_stack_launch_target(backend: str) -> SimStackLaunchTarget:
    normalized_backend = str(backend or "")
    if normalized_backend == "docker":
        return SimStackLaunchTarget(
            backend=normalized_backend,
            start_script=START_DOCKER_SIM_STACK_SCRIPT,
            start_label="Docker SITL/MuJoCo",
        )
    return SimStackLaunchTarget(
        backend=normalized_backend,
        start_script=START_SIM_STACK_SCRIPT,
        start_label="SITL/MuJoCo",
    )


def sim_stack_start_script_error(start_script) -> str | None:
    if not start_script.exists():
        return f"sim script missing: {start_script}"
    if not os.access(start_script, os.X_OK):
        return f"sim script is not executable: {start_script}"
    return None


__all__ = [
    "SimStackLaunchTarget",
    "resolve_sim_stack_launch_target",
    "sim_stack_start_script_error",
]
