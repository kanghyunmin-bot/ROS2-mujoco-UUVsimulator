"""Process start orchestration for GUI-started simulator stacks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .sim_stack_launch_command import build_sim_stack_launch_command
from .sim_stack_launch_logs import open_gui_sim_stack_log
from .sim_stack_launch_process import spawn_sim_stack_process


@dataclass
class StartedSimStackProcess:
    proc: Any
    log_path: Any


def start_gui_sim_stack_process(owner, *, target, extra_args: list[str] | None, env: dict[str, str]) -> StartedSimStackProcess | None:
    try:
        log_path, log_file = open_gui_sim_stack_log()
    except Exception as exc:
        owner._set_sim_stack_status(f"sim log open failed: {exc}")
        return None

    try:
        cmd = build_sim_stack_launch_command(
            owner,
            start_script=target.start_script,
            backend=target.backend,
            extra_args=extra_args,
        )
        proc = spawn_sim_stack_process(cmd=cmd, env=env, log_file=log_file)
    except Exception as exc:
        owner._set_sim_stack_status(f"sim start failed: {exc}")
        return None
    finally:
        _close_log_file(log_file)
    return StartedSimStackProcess(proc=proc, log_path=log_path)


def _close_log_file(log_file) -> None:
    try:
        log_file.close()
    except Exception:
        pass


__all__ = ["StartedSimStackProcess", "start_gui_sim_stack_process"]
