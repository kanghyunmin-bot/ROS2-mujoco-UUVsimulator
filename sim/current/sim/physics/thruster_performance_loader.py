"""Load and select PWM-force curves from thruster performance JSON."""

from __future__ import annotations

from pathlib import Path

from sim.physics.thruster_performance_config import (
    default_thruster_performance_config,
    normalize_thruster_perf_voltage,
)
from sim.physics.thruster_performance_curves import (
    parse_performance_candidates,
    select_nearest_performance_candidate,
    selected_curve_config,
)
from sim.physics.thruster_performance_payload import log_loaded_curve, read_performance_payload


def load_thruster_performance_config(
    path: Path,
    *,
    requested_voltage: float,
    direct: bool,
) -> dict:
    """Load nearest PWM-force curve while preserving the legacy dict contract."""

    perf_cfg = default_thruster_performance_config(
        requested_voltage=requested_voltage,
        direct=direct,
    )
    perf_path = path.expanduser()
    payload = read_performance_payload(perf_path)
    if payload is None:
        return perf_cfg

    candidates = parse_performance_candidates(payload)
    if not candidates:
        print(f"[thruster perf] no usable curve in: {perf_path}", flush=True)
        return perf_cfg

    requested = normalize_thruster_perf_voltage(requested_voltage, requested_voltage)
    selected = select_nearest_performance_candidate(candidates, requested)
    perf_cfg.update(selected_curve_config(selected, requested))
    log_loaded_curve(perf_path, selected_voltage=float(selected["voltage"]), requested=requested, direct=direct)
    return perf_cfg


__all__ = ["load_thruster_performance_config"]
