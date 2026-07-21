"""Process-finish handling for GUI-owned simulator stack logs."""

from __future__ import annotations


def finish_sim_stack_process(owner, proc, rc: int, last_line: str) -> None:
    def finish() -> None:
        if owner._sim_stack_process is proc:
            owner._sim_stack_process = None
            owner._sim_stack_owned_by_gui = False
        if rc == 0:
            owner.sim_stack_status_var.set("sim: exited")
            owner.node.push_event("sim stack exited")
        elif rc < 0:
            owner.sim_stack_status_var.set("sim: stopped")
            owner.node.push_event("sim stack stopped")
        else:
            text = last_line if last_line else f"rc={rc}"
            owner.sim_stack_status_var.set(f"sim failed: {text}")
            owner.node.push_event(f"sim stack failed: {text}")
        owner._refresh_sim_stack_controls()

    try:
        owner.root.after(0, finish)
    except Exception:
        pass


__all__ = ["finish_sim_stack_process"]
