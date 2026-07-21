"""History bookkeeping for synthetic ExternalNav VPD deltas."""

from __future__ import annotations

import numpy as np


def take_vpd_previous_state(self) -> tuple[float | None, np.ndarray | None, np.ndarray | None]:
    return (
        self._sitl_vpd_prev_clock_t,
        self._sitl_vpd_prev_pos_ned,
        self._sitl_vpd_prev_rot_ned_bfrd,
    )


def store_vpd_current_state(
    self,
    *,
    vpd_clock_t: float,
    pos_ned: np.ndarray,
    rot_ned_bfrd: np.ndarray,
) -> None:
    self._sitl_vpd_prev_clock_t = float(vpd_clock_t)
    self._sitl_vpd_prev_pos_ned = pos_ned[:3].copy()
    self._sitl_vpd_prev_rot_ned_bfrd = rot_ned_bfrd.copy()


def warn_stale_vpd_sample(self, *, dt_s: float, now_wall: float) -> None:
    if self._sitl_cmd_debug and now_wall - self._sitl_extnav_last_log_wall >= 2.0:
        print(
            "[sitl_transport] ExternalNav VPD history reset after stale sample "
            f"dt={dt_s:.3f}s",
            flush=True,
        )
        self._sitl_extnav_last_log_wall = now_wall


__all__ = [
    "store_vpd_current_state",
    "take_vpd_previous_state",
    "warn_stale_vpd_sample",
]
