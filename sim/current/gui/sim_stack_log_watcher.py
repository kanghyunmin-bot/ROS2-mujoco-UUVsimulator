"""GUI-owned simulator stack log watcher."""

from __future__ import annotations

from .runtime import subprocess
from .sim_stack_log_finish import finish_sim_stack_process
from .sim_stack_log_line_handler import handle_sim_stack_log_line
from .sim_stack_log_reader import follow_process_log


class SimStackLogWatcherMixin:
    def _watch_sim_stack_output(self, proc: subprocess.Popen[str], log_path) -> None:
        rc, last_line = follow_process_log(proc, log_path, self._handle_sim_stack_log_line)
        self._finish_sim_stack_process(proc, rc, last_line)

    def _handle_sim_stack_log_line(self, raw_line: str, last_line: str) -> str:
        return handle_sim_stack_log_line(self, raw_line, last_line)

    def _finish_sim_stack_process(self, proc: subprocess.Popen[str], rc: int, last_line: str) -> None:
        finish_sim_stack_process(self, proc, rc, last_line)


__all__ = ["SimStackLogWatcherMixin"]
