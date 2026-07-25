"""Wait-loop helpers for MAVROS roll-stability probe commands."""

from __future__ import annotations

from collections.abc import Callable

from roll_stability_wait_loop import deadline_after, neutral_spin_once, spin_once, wait_until


def wait_for_stack_ready(node: object, *, timeout: float) -> None:
    def poll_ready() -> bool:
        services_ok = node.arm_client.wait_for_service(timeout_sec=0.15) and node.mode_client.wait_for_service(
            timeout_sec=0.15
        )
        spin_once(node)
        state_ok = node.state is not None and bool(node.state.connected)
        return services_ok and state_ok

    if wait_until(deadline_after(timeout), poll_ready):
        return
    raise RuntimeError("ROS2/MAVROS services or connected state did not become ready")


def wait_future_done_with_neutral(node: object, future: object, *, timeout: float, timeout_message: str) -> None:
    def poll_done() -> bool:
        if future.done():
            return True
        neutral_spin_once(node)
        return future.done()

    if not wait_until(deadline_after(timeout), poll_done):
        raise RuntimeError(timeout_message)


def wait_state_condition_with_neutral(
    node: object,
    *,
    timeout: float,
    predicate: Callable[[], bool],
) -> bool:
    def poll_predicate() -> bool:
        neutral_spin_once(node)
        return predicate()

    return wait_until(deadline_after(timeout), poll_predicate)


__all__ = [
    "wait_for_stack_ready",
    "wait_future_done_with_neutral",
    "wait_state_condition_with_neutral",
]
