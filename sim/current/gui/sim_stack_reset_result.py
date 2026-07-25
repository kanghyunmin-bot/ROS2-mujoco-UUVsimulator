"""GUI state updates for simulator reset results."""

from __future__ import annotations


def apply_reset_result(gui, rc: int) -> None:
    if int(rc) == 0:
        if gui._wait_for_external_sim_stack_exit(timeout_s=3.0):
            gui._sim_stack_owned_by_gui = False
            gui._set_sim_stack_status("sim: stopped/reset")
            gui.node.push_event("sim stack stopped/reset")
        else:
            gui._set_sim_stack_status("sim: reset done; external process still running")
            gui.node.push_event("sim reset done; external process still running")
    else:
        gui._set_sim_stack_status(f"sim reset failed: rc={rc}")
        gui.node.push_event(f"sim reset failed: rc={rc}")


def refresh_sim_stack_controls_async(gui) -> None:
    try:
        gui.root.after(0, gui._refresh_sim_stack_controls)
    except Exception:
        pass


__all__ = ["apply_reset_result", "refresh_sim_stack_controls_async"]
