"""Runtime writer facade for MuJoCo thruster debug CSV rows."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TextIO

from sim.runtime.thruster_debug_file import open_thruster_debug_file
from sim.runtime.thruster_debug_runtime_emit import emit_thruster_debug_if_due


@dataclass
class ThrusterDebugRuntime:
    """Own the optional thruster debug stream outside the main run loop."""

    file: TextIO | None
    thruster_names: list[str]
    next_sample_t: float = 0.0

    @classmethod
    def create(
        cls,
        *,
        path_text: str,
        thruster_names: list[str],
        log: Callable[[str], None],
    ) -> "ThrusterDebugRuntime":
        """Create a CSV writer when the debug path is configured."""
        debug_file = open_thruster_debug_file(
            path_text=path_text,
            thruster_names=thruster_names,
            log=log,
        )
        return cls(file=debug_file, thruster_names=list(thruster_names))

    @property
    def enabled(self) -> bool:
        return self.file is not None

    def close(self) -> None:
        """Close the stream exactly once."""
        if self.file is None:
            return
        self.file.close()
        self.file = None

    def emit(self, **payload: object) -> None:
        """Write one 20 Hz debug sample when enabled."""
        emit_thruster_debug_if_due(self, **payload)


__all__ = ["ThrusterDebugRuntime"]
