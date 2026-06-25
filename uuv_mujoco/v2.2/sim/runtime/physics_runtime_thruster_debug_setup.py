"""Create optional thruster debug logging runtime."""

from __future__ import annotations

import os

from sim.runtime.thruster_debug_runtime import ThrusterDebugRuntime


def create_thruster_debug_runtime(*, all_thruster_names: list[str], log) -> ThrusterDebugRuntime:
    return ThrusterDebugRuntime.create(
        path_text=os.environ.get("UUV_MJ_THRUSTER_DEBUG_CSV", ""),
        thruster_names=all_thruster_names,
        log=log,
    )


__all__ = ["create_thruster_debug_runtime"]
