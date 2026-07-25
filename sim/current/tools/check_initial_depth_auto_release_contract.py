#!/usr/bin/env python3
"""Smoke checks for initial-depth hold auto-release policy."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.simulation_step_hold_release import maybe_release_initial_depth_hold  # noqa: E402


class FakeBridge:
    def __init__(self, *, armed: bool, mode: str) -> None:
        self._armed = bool(armed)
        self._mode = str(mode)

    def sitl_vehicle_armed(self) -> bool:
        return self._armed

    def sitl_vehicle_mode(self) -> str:
        return self._mode


def _released_reason(*, mode: str, pwm: list[int], armed: bool = True, active: bool = True) -> list[str]:
    reasons: list[str] = []
    maybe_release_initial_depth_hold(
        initial_depth_hold={"active": bool(active)},
        auto_release=True,
        ros_bridge=FakeBridge(armed=armed, mode=mode),
        sitl_servo_pwm_values=pwm,
        release_initial_depth_hold=lambda reason: reasons.append(str(reason)),
    )
    return reasons


def main() -> int:
    saturated_vertical = [1500, 1500, 1500, 1500, 1100, 1900, 1900, 1100]
    manual_nonneutral = [1500, 1500, 1500, 1650, 1500, 1500, 1500, 1500]

    if _released_reason(mode="ALT_HOLD", pwm=saturated_vertical):
        raise AssertionError("ALT_HOLD controller saturation must not auto-release initial-depth hold")
    if _released_reason(mode="MANUAL", pwm=[1500] * 8):
        raise AssertionError("neutral MANUAL servo output must not auto-release initial-depth hold")
    if _released_reason(mode="MANUAL", pwm=manual_nonneutral) != ["auto:armed_nonneutral_servo"]:
        raise AssertionError("nonneutral MANUAL servo output must auto-release initial-depth hold")
    if _released_reason(mode="MANUAL", pwm=manual_nonneutral, armed=False):
        raise AssertionError("disarmed vehicle must not auto-release initial-depth hold")

    print("initial_depth_auto_release_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
