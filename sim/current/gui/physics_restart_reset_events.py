"""GUI event helpers for physics-profile reset restarts."""

from __future__ import annotations


def publish_physics_reset_log_lines(self, stdout: str) -> None:
    for raw_line in stdout.splitlines():
        line = raw_line.strip()
        if line.startswith("[reset]"):
            short_line = line if len(line) <= 150 else f"{line[:147]}..."
            self.node.push_event(short_line)


def handle_external_stack_still_running(self) -> None:
    self._set_sim_stack_status("sim: external stack still running after reset")
    self.node.push_event("physics params restart blocked: external stack still running")
    try:
        self.root.after(0, self._refresh_sim_stack_controls)
    except Exception:
        pass


__all__ = [
    "handle_external_stack_still_running",
    "publish_physics_reset_log_lines",
]
