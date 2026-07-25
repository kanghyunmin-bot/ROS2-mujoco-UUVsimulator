"""Initial-depth contract compatibility exports for GUI-started simulator stacks."""

from __future__ import annotations

from .sim_stack_initial_depth_base_link import (
    base_link_debug_initial_depth_contract,
    base_link_initial_depth_event,
)
from .sim_stack_initial_depth_sources import (
    bar30_initial_depth_contract,
    initial_depth_args_explicitly_set,
    real_start_initial_depth_contract,
)


__all__ = [
    "bar30_initial_depth_contract",
    "base_link_debug_initial_depth_contract",
    "base_link_initial_depth_event",
    "initial_depth_args_explicitly_set",
    "real_start_initial_depth_contract",
]
