"""Simulator stack process controls for the GUI."""

from __future__ import annotations

from .process_common_mixin import ProcessCommonMixin
from .sim_stack_launch_mixin import SimStackLaunchMixin
from .sim_stack_reset_mixin import SimStackResetMixin
from .sim_stack_status_mixin import SimStackStatusMixin


class SimStackProcessMixin(
    SimStackLaunchMixin,
    SimStackResetMixin,
    SimStackStatusMixin,
    ProcessCommonMixin,
):
    """Compatibility mixin preserving the simulator stack process API."""


__all__ = ["SimStackProcessMixin"]
