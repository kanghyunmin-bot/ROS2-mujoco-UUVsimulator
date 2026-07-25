"""T200 thruster performance curve loader used by offline replay tools."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .thruster_performance_curves import (
    parse_thruster_performance_candidates,
    select_nearest_thruster_performance_candidate,
)
from .thruster_performance_payload import read_thruster_performance_payload


@dataclass(frozen=True)
class ThrusterPerformance:
    active: bool
    requested_voltage: float
    selected_voltage: float | None
    pwm_us: np.ndarray
    force_n: np.ndarray

    @property
    def force_max(self) -> float | None:
        if not self.active or self.force_n.size == 0:
            return None
        return float(np.max(np.abs(self.force_n)))

    def force_from_norm(self, norm_cmd: float) -> float:
        if not self.active or self.pwm_us.size < 2:
            return 0.0
        pwm = float(np.clip(float(norm_cmd), -1.0, 1.0) * 400.0 + 1500.0)
        return float(np.interp(pwm, self.pwm_us, self.force_n))


def load_thruster_performance(path: Path, requested_voltage: float) -> tuple[ThrusterPerformance, str | None]:
    requested = float(requested_voltage)
    inactive = ThrusterPerformance(False, requested, None, np.array([], dtype=np.float64), np.array([], dtype=np.float64))
    perf_path = Path(path).expanduser()
    payload, error = read_thruster_performance_payload(perf_path)
    if payload is None:
        return inactive, error

    candidates = parse_thruster_performance_candidates(payload["curves"])

    if not candidates:
        return inactive, f"[thruster perf] no usable curve in: {perf_path}"

    selected = select_nearest_thruster_performance_candidate(candidates, requested)
    cfg = ThrusterPerformance(
        True,
        requested,
        float(selected["voltage"]),
        np.asarray(selected["pwm"], dtype=np.float64),
        np.asarray(selected["force"], dtype=np.float64),
    )
    return cfg, (
        f"[thruster perf] loaded curve {cfg.selected_voltage}V from {perf_path} "
        f"(requested {requested}V)"
    )
