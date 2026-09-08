"""Runtime emission policy for MuJoCo thruster debug CSV rows."""

from __future__ import annotations

from typing import Any

from sim.runtime.thruster_debug_emit import emit_thruster_debug_row
from sim.runtime.thruster_debug_schedule import (
    advance_thruster_debug_sample_time,
    should_emit_thruster_debug_sample,
)


def emit_thruster_debug_if_due(runtime: Any, **payload: Any) -> None:
    """Write one debug sample when the runtime stream and 100 Hz slot are ready."""
    if runtime.file is None:
        return
    data = payload["data"]
    sim_t = float(data.time)
    if not should_emit_thruster_debug_sample(
        sim_t=sim_t,
        next_sample_t=runtime.next_sample_t,
    ):
        return
    runtime.next_sample_t = advance_thruster_debug_sample_time(
        sim_t=sim_t,
        next_sample_t=runtime.next_sample_t,
    )

    emit_thruster_debug_row(
        runtime.file,
        thruster_names=runtime.thruster_names,
        **payload,
    )


__all__ = ["emit_thruster_debug_if_due"]
