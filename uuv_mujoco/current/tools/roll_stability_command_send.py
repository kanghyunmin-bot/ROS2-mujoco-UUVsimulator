"""MAVROS arm and mode command send helpers for roll-stability probes."""

from __future__ import annotations

from roll_stability_command_requests import build_arm_request, build_set_mode_request
from roll_stability_command_wait import wait_future_done_with_neutral, wait_state_condition_with_neutral


def set_mode_via_mavros(node, mode: str, *, timeout: float) -> None:
    req = build_set_mode_request(mode)
    future = node.mode_client.call_async(req)
    wait_future_done_with_neutral(
        node,
        future,
        timeout=timeout,
        timeout_message=f"set_mode({mode}) timeout",
    )
    response = future.result()
    response_ok = response is not None and bool(response.mode_sent)
    if wait_state_condition_with_neutral(
        node,
        timeout=timeout,
        predicate=lambda: node.state is not None and str(node.state.mode) == mode,
    ):
        return
    if not response_ok:
        raise RuntimeError(f"set_mode({mode}) failed")
    raise RuntimeError(f"state did not report mode {mode}")


def arm_via_mavros(node, value: bool, *, timeout: float) -> None:
    req = build_arm_request(value)
    future = node.arm_client.call_async(req)
    wait_future_done_with_neutral(
        node,
        future,
        timeout=timeout,
        timeout_message=f"arming({value}) timeout",
    )
    response = future.result()
    response_ok = response is not None and bool(response.success)
    if wait_state_condition_with_neutral(
        node,
        timeout=timeout,
        predicate=lambda: node.state is not None and bool(node.state.armed) == bool(value),
    ):
        return
    if not response_ok:
        raise RuntimeError(f"arming({value}) failed")
    raise RuntimeError(f"state did not report armed={value}")


__all__ = ["arm_via_mavros", "set_mode_via_mavros"]
