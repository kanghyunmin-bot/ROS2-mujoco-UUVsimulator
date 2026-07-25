"""Validation helpers for ArduSub JSON sensor packets."""

from __future__ import annotations

import numpy as np


def sitl_json_payload_is_finite(payload: dict[str, object]) -> bool:
    arrs = (
        np.array(payload["imu"]["gyro"], dtype=np.float64),
        np.array(payload["imu"]["accel_body"], dtype=np.float64),
        np.array(payload["velocity"], dtype=np.float64),
        np.array(payload["position"], dtype=np.float64),
        np.array(payload["quaternion"], dtype=np.float64),
        np.array(payload["attitude"], dtype=np.float64),
    )
    return bool(np.isfinite(float(payload["timestamp"])) and all(np.all(np.isfinite(a)) for a in arrs))


def warn_nonfinite_sitl_json_payload_once(self) -> None:
    if not self._sitl_nonfinite_warned:
        self._sitl_nonfinite_warned = True
        print("[sitl_transport] skip SITL packet: non-finite sensor value", flush=True)


__all__ = ["sitl_json_payload_is_finite", "warn_nonfinite_sitl_json_payload_once"]
