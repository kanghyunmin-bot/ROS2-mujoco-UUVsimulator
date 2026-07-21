"""Service and command-wait compatibility exports for axis RC checks."""

from __future__ import annotations

from axis_rc_arm_service import call_arm
from axis_rc_mode_service import call_set_mode
from axis_rc_stack_wait import wait_for_stack
from axis_rc_trigger_service import call_trigger_service


__all__ = ["call_arm", "call_set_mode", "call_trigger_service", "wait_for_stack"]
