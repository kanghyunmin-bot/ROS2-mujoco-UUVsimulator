"""Shutdown helpers for the UUV control GUI application."""

from __future__ import annotations

from collections.abc import Callable

from .runtime import rclpy


def _ignore_errors(action: Callable[[], object]) -> None:
    try:
        action()
    except Exception:
        pass


def cancel_scheduled_update(owner) -> None:
    if owner._after_id is None:
        pass
    else:
        _ignore_errors(lambda: owner.root.after_cancel(owner._after_id))
        owner._after_id = None
    rc_fast_after_id = getattr(owner, "_rc_fast_after_id", None)
    if rc_fast_after_id is not None:
        _ignore_errors(lambda: owner.root.after_cancel(rc_fast_after_id))
        owner._rc_fast_after_id = None


def stop_rc_replay(owner) -> None:
    owner._rc_replay_stop_event.set()
    owner._rc_replay_pause_event.clear()


def terminate_child_processes(owner) -> None:
    for proc in (owner._ros_pkg_process, owner._ros_build_process, owner._rviz_process):
        owner._terminate_process_group(proc)


def sim_stack_owned_by_gui(owner) -> bool:
    if owner._sim_stack_owned_by_gui:
        return True
    return owner._sim_stack_process is not None and owner._sim_stack_process.poll() is None


def stop_owned_sim_stack(owner) -> None:
    owned_sim_stack = sim_stack_owned_by_gui(owner)
    owner._terminate_sim_stack_process()
    if owned_sim_stack:
        owner._reset_sim_stack_blocking()


def publish_rc_release(owner) -> None:
    _ignore_errors(owner.node.publish_rc_release)


def shutdown_ros_runtime(owner) -> None:
    _ignore_errors(owner._executor.shutdown)
    _ignore_errors(owner.node.destroy_node)
    if rclpy.ok():
        _ignore_errors(rclpy.shutdown)


def destroy_root(owner) -> None:
    _ignore_errors(owner.root.destroy)


__all__ = [
    "cancel_scheduled_update",
    "destroy_root",
    "publish_rc_release",
    "shutdown_ros_runtime",
    "stop_owned_sim_stack",
    "stop_rc_replay",
    "terminate_child_processes",
]
