"""Thread-safe direct command state for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass, field
import threading

from sim.runtime.command_state_store import (
    clamped_command_update,
    command_recently_active,
    default_command_values,
    normalized_command_snapshot,
)
from sim.runtime.command_state_values import clamp_command_value


@dataclass
class RuntimeCommandState:
    """Store the latest direct command with timeout and normalization helpers."""

    max_value: float = 15.0
    step: float = 2.0
    timeout_s: float = 0.45
    last_wall: float = -1.0
    _values: dict[str, float] = field(default_factory=default_command_values)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    @staticmethod
    def clamp(value: float, max_value: float) -> float:
        """Clamp a command to the symmetric runner command limit."""
        return clamp_command_value(value, max_value)

    def update(
        self,
        *,
        forward: float,
        sway: float,
        yaw: float,
        heave: float,
        now_wall: float,
        allow: bool = True,
    ) -> bool:
        """Update the latest command if the direct command path is enabled."""
        if not allow:
            return False
        with self._lock:
            self._values.update(
                clamped_command_update(
                    forward=forward,
                    sway=sway,
                    yaw=yaw,
                    heave=heave,
                    max_value=self.max_value,
                )
            )
            self.last_wall = float(now_wall)
        return True

    def normalized(self) -> tuple[float, float, float, float]:
        """Return normalized forward, sway, yaw, heave commands."""
        with self._lock:
            values = dict(self._values)
        return normalized_command_snapshot(values, max_value=self.max_value)

    def recently_active(self, now_wall: float) -> bool:
        """Return true if a direct command was received within the timeout."""
        return command_recently_active(
            last_wall=self.last_wall,
            now_wall=now_wall,
            timeout_s=self.timeout_s,
        )


__all__ = ["RuntimeCommandState"]
