"""T200 thruster performance curve loader used by offline replay tools."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


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


def _to_float_array(values: Any) -> np.ndarray | None:
    if not isinstance(values, list) or not values:
        return None
    try:
        out = np.array([float(value) for value in values], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(out)):
        return None
    return out


def load_thruster_performance(path: Path, requested_voltage: float) -> tuple[ThrusterPerformance, str | None]:
    requested = float(requested_voltage)
    inactive = ThrusterPerformance(False, requested, None, np.array([], dtype=np.float64), np.array([], dtype=np.float64))
    perf_path = Path(path).expanduser()
    if not perf_path.exists():
        return inactive, f"[thruster perf] file not found: {perf_path}"
    try:
        payload = json.loads(perf_path.read_text())
    except (OSError, json.JSONDecodeError):
        return inactive, f"[thruster perf] invalid json: {perf_path}"

    curves_raw = payload.get("curves") if isinstance(payload, dict) else None
    if not isinstance(curves_raw, list):
        return inactive, f"[thruster perf] missing curves in: {perf_path}"

    candidates: list[dict[str, Any]] = []
    for curve in curves_raw:
        if not isinstance(curve, dict):
            continue
        voltage = curve.get("voltage_v")
        pwm = _to_float_array(curve.get("pwm_us"))
        force = _to_float_array(curve.get("force_n"))
        if voltage is None or pwm is None or force is None or pwm.size != force.size or pwm.size < 2:
            continue
        order = np.argsort(pwm)
        pwm = pwm[order]
        force = force[order]
        valid = np.isfinite(pwm) & np.isfinite(force)
        if np.sum(valid) < 2:
            continue
        candidates.append({"voltage": float(voltage), "pwm": pwm[valid], "force": force[valid]})

    if not candidates:
        return inactive, f"[thruster perf] no usable curve in: {perf_path}"

    selected = min(candidates, key=lambda item: abs(float(item["voltage"]) - requested))
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
