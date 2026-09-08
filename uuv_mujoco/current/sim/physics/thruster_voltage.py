"""Measured PWM/voltage surface and optional ESC-bus voltage recordings.

Voltage interpolation is restricted to measured data. A battery trace contains
terminal voltage at the ESC, including sag; no invented battery resistance or
state-of-charge curve is applied on top of that measurement.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np


def measured_voltage_surface(payload: dict) -> dict:
    curves = []
    for raw in payload["curves"]:
        if not isinstance(raw, dict):
            continue
        if "extrapolat" in str(raw.get("source", "")).lower():
            continue
        voltage = float(raw["voltage_v"])
        # The bundled manufacturer's data is measured at 10--20 V only.
        if not 10.0 <= voltage <= 20.0:
            continue
        pwm = np.asarray(raw["pwm_us"], dtype=float)
        force = np.asarray(raw["force_n"], dtype=float)
        if (pwm.ndim != 1 or pwm.size < 2 or force.shape != pwm.shape
                or not np.all(np.isfinite(pwm)) or not np.all(np.isfinite(force))):
            raise ValueError("invalid measured thruster curve")
        order = np.argsort(pwm)
        pwm, force = pwm[order], force[order]
        if np.any(np.diff(pwm) <= 0) or pwm[0] > 1100 or pwm[-1] < 1900:
            raise ValueError("thruster curve must uniquely cover 1100--1900 us")
        if abs(float(np.interp(1500, pwm, force))) > 1e-9:
            raise ValueError("thruster curve must have zero neutral thrust")
        if np.any(force[pwm < 1500] > 1e-9) or np.any(force[pwm > 1500] < -1e-9):
            raise ValueError("thruster curve has inconsistent forward/reverse signs")
        curves.append((voltage, pwm, force))
    curves.sort(key=lambda c: c[0])
    if not curves or any(a[0] == b[0] for a, b in zip(curves, curves[1:])):
        raise ValueError("expected unique measured T200 voltage curves")
    grid = np.unique(np.concatenate([c[1] for c in curves]))
    return {
        "voltage_grid": np.array([c[0] for c in curves]),
        "pwm": grid,
        "force_surface": np.array([np.interp(grid, c[1], c[2]) for c in curves]),
    }


def force_curve_at_voltage(config: dict, voltage: float) -> np.ndarray:
    grid = config["voltage_grid"]
    if not np.isfinite(voltage) or not grid[0] <= voltage <= grid[-1]:
        raise ValueError(f"ESC bus voltage {voltage} V outside measured T200 range {grid[0]}--{grid[-1]} V")
    hi = min(int(np.searchsorted(grid, voltage)), len(grid) - 1)
    lo = max(0, hi - 1)
    if hi == lo:
        return config["force_surface"][lo].copy()
    fraction = (voltage - grid[lo]) / (grid[hi] - grid[lo])
    return (1 - fraction) * config["force_surface"][lo] + fraction * config["force_surface"][hi]


def load_voltage_trace(path: Path, config: dict) -> dict:
    with path.expanduser().open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    samples = np.array([(float(r["sim_time"]), float(r["voltage_v"])) for r in rows])
    if (samples.ndim != 2 or len(samples) < 2 or not np.all(np.isfinite(samples))
            or samples[0, 0] != 0 or np.any(np.diff(samples[:, 0]) <= 0)):
        raise ValueError("voltage CSV requires increasing sim_time starting at 0 and at least two finite samples")
    for voltage in samples[:, 1]:
        force_curve_at_voltage(config, float(voltage))
    return {"time": samples[:, 0], "voltage": samples[:, 1]}


def update_supply_voltage(config: dict, sim_time: float) -> None:
    trace = config.get("voltage_trace")
    if trace is None:
        return
    # Hold endpoints outside the trace; log this policy when loading the file.
    voltage = float(np.interp(sim_time, trace["time"], trace["voltage"]))
    if voltage != config["selected_voltage"]:
        config["force"] = force_curve_at_voltage(config, voltage)
        config["selected_voltage"] = voltage
