"""Vehicle state and arm/mode gate helpers for GUI commands."""

from __future__ import annotations

from .readiness_contract import arm_mode_gate_reason
from .runtime import Optional, math, time


def _state_age_s(self) -> float:
    return time.monotonic() - self._last_wall.get("state", math.inf)


def _fresh_vehicle_state(self) -> tuple[bool, bool, str]:
    with self._lock:
        return bool(self._snapshot.connected), bool(self._snapshot.armed), str(self._snapshot.mode)


def _arm_mode_settle_left_s(self) -> float:
    if not self._require_arm_mode_settle:
        return 0.0
    with self._lock:
        connected_since = float(self._vehicle_connected_since_wall)
    if connected_since <= 0.0:
        return float(max(0.0, self._arm_mode_settle_s))
    age_s = time.monotonic() - connected_since
    return float(max(0.0, self._arm_mode_settle_s - age_s))


def _arm_mode_gate_reason(self, *, arm_value: Optional[bool] = None, mode: str = "") -> str:
    return arm_mode_gate_reason(
        self._effective_backend(),
        self.snapshot(),
        settle_left_s=self._arm_mode_settle_left_s(),
        arm_value=arm_value,
        mode=mode,
    )


def _arm_target_reached(self, value: bool) -> bool:
    connected, armed, _mode = self._fresh_vehicle_state()
    return connected and self._state_age_s() < 3.0 and armed == bool(value)


def _mode_target_reached(self, mode: str) -> bool:
    connected, _armed, current_mode = self._fresh_vehicle_state()
    return connected and self._state_age_s() < 3.0 and current_mode.upper() == str(mode).upper()


__all__ = [
    "_arm_mode_gate_reason",
    "_arm_mode_settle_left_s",
    "_arm_target_reached",
    "_fresh_vehicle_state",
    "_mode_target_reached",
    "_state_age_s",
]
