"""State transitions for GUI-started simulator stacks."""

from __future__ import annotations

from .sim_stack_launch_process import start_sim_stack_watcher


def prepare_gui_sim_stack_start(owner) -> None:
    if owner._rc_replay_running():
        owner._stop_rc_replay()
    owner.rc_override_enabled.set(False)
    owner.node.publish_rc_release()


def record_gui_sim_stack_started(owner, *, target, started) -> None:
    owner._sim_stack_process = started.proc
    owner._sim_stack_log_path = started.log_path
    owner._external_sim_stack_running_cached = False
    owner._sim_stack_owned_by_gui = True
    owner._set_sim_stack_status(f"sim: starting {target.start_label}")
    owner._refresh_sim_stack_controls()
    owner.node.push_event(f"sim stack start requested ({target.backend}): {started.log_path.name}")
    owner._sim_stack_thread = start_sim_stack_watcher(
        owner,
        proc=started.proc,
        log_path=started.log_path,
    )


__all__ = ["prepare_gui_sim_stack_start", "record_gui_sim_stack_started"]
