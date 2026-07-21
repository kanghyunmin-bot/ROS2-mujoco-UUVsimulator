"""Service wrapper helpers for axis RC checks."""

from __future__ import annotations

from typing import Any

from axis_rc_services import (
    call_arm as axis_call_arm,
    call_set_mode as axis_call_set_mode,
    call_trigger_service as axis_call_trigger_service,
    wait_for_stack as axis_wait_for_stack,
)


class AxisRcNodeServices:
    def wait_for_stack(self, timeout: float, *, require_manual_input: bool = False) -> None:
        axis_wait_for_stack(self, timeout, require_manual_input=require_manual_input)

    def call_set_mode(self, mode: str, timeout: float = 10.0) -> None:
        axis_call_set_mode(self, mode, timeout=timeout)

    def call_arm(self, armed: bool, timeout: float = 10.0) -> None:
        axis_call_arm(self, armed, timeout=timeout)

    def call_trigger_service(self, client: Any, service_name: str, timeout: float = 10.0) -> None:
        axis_call_trigger_service(self, client, service_name, timeout=timeout)

    def switch_initial_depth_hold_to_target(self, timeout: float = 10.0) -> None:
        self.call_trigger_service(
            self.switch_initial_depth_hold_client,
            "/mujoco/switch_initial_depth_hold_to_target",
            timeout=timeout,
        )

    def release_initial_depth_hold(self, timeout: float = 10.0) -> None:
        self.call_trigger_service(
            self.release_initial_depth_hold_client,
            "/mujoco/release_initial_depth_hold",
            timeout=timeout,
        )


__all__ = ["AxisRcNodeServices"]
