"""Handler registration and external plant override helpers for SitlTransport."""

from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np


def set_servo_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
    self._sitl_servo_callback = callback
    print("[sitl_transport] SITL control mode set: servo-direct", flush=True)


def set_servo_telemetry_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
    """Register a handler for MAVLink SERVO_OUTPUT_RAW telemetry only.

    This is intentionally separate from the active plant command callback:
    closed_loop plant input uses the raw JSON servo backend, while controller
    parity observations must use MAVLink SERVO_OUTPUT_RAW.
    """
    self._sitl_servo_telemetry_callback = callback


def inject_servo_pwm_values(
    self,
    pwm_values: list[int],
    *,
    hold_s: float = 1.0,
    source: str = "replay_rcout",
) -> None:
    """Inject recorded SERVO_OUTPUT_RAW values as the active plant command."""
    now_wall = time.monotonic()
    if not self._allow_rcout_plant_override:
        if now_wall - self._sitl_last_rc_override_warn_wall > 3.0:
            print(
                "[sitl_transport] RCOUT plant override rejected by "
                "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE=0",
                flush=True,
            )
            self._sitl_last_rc_override_warn_wall = now_wall
        return
    hold_s = float(np.clip(float(hold_s), 0.05, 5.0))
    self._sitl_external_servo_override_until_wall = now_wall + hold_s
    if now_wall - self._sitl_external_servo_override_log_wall > 3.0:
        print(
            f"[sitl_transport] external SERVO_OUTPUT_RAW override active "
            f"source={source} hold_s={hold_s:.2f}",
            flush=True,
        )
        self._sitl_external_servo_override_log_wall = now_wall
    self._handle_pwm_values([int(v) for v in pwm_values], now_wall, source=source)


__all__ = ["inject_servo_pwm_values", "set_servo_handler", "set_servo_telemetry_handler"]
