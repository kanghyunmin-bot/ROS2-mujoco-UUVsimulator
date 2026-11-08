"""Load and select PWM-force curves from thruster performance JSON."""

from __future__ import annotations

from pathlib import Path

from sim.physics.thruster_performance_config import (
    default_thruster_performance_config,
    normalize_thruster_perf_voltage,
)
from sim.physics.thruster_performance_payload import log_loaded_curve, read_performance_payload
from sim.physics.thruster_voltage import measured_voltage_surface, select_supply_voltage


def load_thruster_performance_config(
    path: Path,
    *,
    requested_voltage: float,
    direct: bool,
    include_rpm: bool = False,
) -> dict:
    """Interpolate the measured PWM/voltage surface without extrapolation."""

    perf_cfg = default_thruster_performance_config(
        requested_voltage=requested_voltage,
        direct=direct,
    )
    perf_path = path.expanduser()
    payload = read_performance_payload(perf_path)
    if payload is None:
        raise ValueError(f"requested thruster performance data unavailable: {perf_path}")

    requested = normalize_thruster_perf_voltage(requested_voltage, requested_voltage)
    perf_cfg.update(measured_voltage_surface(payload, include_rpm=include_rpm))
    perf_cfg.update(active=True, requested_voltage=requested)
    select_supply_voltage(perf_cfg, requested)
    log_loaded_curve(perf_path, selected_voltage=requested, requested=requested, direct=direct)
    return perf_cfg


__all__ = ["load_thruster_performance_config"]
