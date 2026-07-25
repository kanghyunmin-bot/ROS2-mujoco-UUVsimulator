"""Process helpers for GUI-started simulator stacks."""

from __future__ import annotations

import subprocess
import threading

from .config import SIM_STACK_DIR


def spawn_sim_stack_process(*, cmd: list[str], env: dict[str, str], log_file):
    return subprocess.Popen(
        cmd,
        cwd=str(SIM_STACK_DIR),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
        start_new_session=True,
    )


def start_sim_stack_watcher(owner, *, proc, log_path):
    thread = threading.Thread(
        target=owner._watch_sim_stack_output,
        args=(proc, log_path),
        daemon=True,
    )
    thread.start()
    return thread


__all__ = ["spawn_sim_stack_process", "start_sim_stack_watcher"]
