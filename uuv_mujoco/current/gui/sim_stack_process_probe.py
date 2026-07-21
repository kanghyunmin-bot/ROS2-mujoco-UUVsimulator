"""Simulator-stack process detection helpers."""

from __future__ import annotations

import time
from collections.abc import Callable


SIM_STACK_COMMAND_PATTERNS = (
    r"(^|[/ ]|\./)start_docker_sitl_mujoco_mj311\.sh($| )",
    r"(^|[/ ]|\./)start_sitl_mujoco_mj311\.sh($| )",
    r"(^|[/ ]|\./)launch_uuv_sim\.sh .*--sitl",
    r"(^|[/ ]|\./)run_uuv_mujoco\.py .*--sitl",
    r"(^|[/ ]|\./)run_urdf_full\.py .*--sitl",
)


ProcessScanner = Callable[[tuple[str, ...]], list[str]]


def external_sim_stack_commands(scanner: ProcessScanner) -> list[str]:
    return scanner(SIM_STACK_COMMAND_PATTERNS)


def wait_for_external_sim_stack_exit(
    scanner: ProcessScanner,
    *,
    timeout_s: float = 10.0,
    poll_period_s: float = 0.2,
) -> tuple[bool, bool]:
    """Return ``(exited, still_running)`` after waiting for external stack exit."""
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        if not external_sim_stack_commands(scanner):
            return True, False
        time.sleep(float(poll_period_s))
    still_running = bool(external_sim_stack_commands(scanner))
    return not still_running, still_running


__all__ = [
    "SIM_STACK_COMMAND_PATTERNS",
    "ProcessScanner",
    "external_sim_stack_commands",
    "wait_for_external_sim_stack_exit",
]
