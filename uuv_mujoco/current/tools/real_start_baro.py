"""Bar30/AP_Baro pressure datum helpers for real-start state extraction."""

from __future__ import annotations

import sys
from pathlib import Path

from real_start_baro_candidates import (
    collect_baro_ground_pressure_candidates,
    select_baro_ground_pressure_candidate,
)

V22_ROOT = Path(__file__).resolve().parents[1]
if str(V22_ROOT) not in sys.path:
    sys.path.insert(0, str(V22_ROOT))

from sim.contracts import (  # noqa: E402
    AP_BARO_FRONTEND_PA_PER_M,
    BaroPressureLaw,
    surface_pressure_for_depth_sample,
)

AP_BARO_SITL_GROUND_PRESSURE_PA = 101473.796875


def infer_baro_real_ground_pressure(rows: list[dict[str, str]], time_col: str) -> tuple[float, str]:
    """Infer the real AP_Baro water frontend pressure datum from the CSV."""
    candidates = collect_baro_ground_pressure_candidates(
        rows,
        time_col=time_col,
        frontend_pa_per_m=AP_BARO_FRONTEND_PA_PER_M,
    )
    return select_baro_ground_pressure_candidate(candidates)


def baro_json_depth_for_frontend_match(pressure_pa: float, real_ground_pressure_pa: float) -> float:
    law = BaroPressureLaw(
        real_ground_pressure_pa=float(real_ground_pressure_pa),
        sitl_ground_pressure_pa=AP_BARO_SITL_GROUND_PRESSURE_PA,
    )
    return law.sitl_depth_m_for_frontend_match(pressure_pa)


def baro_frontend_depth_m(pressure_pa: float, real_ground_pressure_pa: float) -> float:
    """Positive-down depth implied by ArduPilot's water baro frontend."""
    law = BaroPressureLaw(
        real_ground_pressure_pa=float(real_ground_pressure_pa),
        sitl_ground_pressure_pa=AP_BARO_SITL_GROUND_PRESSURE_PA,
    )
    return law.frontend_depth_m_from_pressure(pressure_pa)


def surface_pressure_for_sample(pressure_pa: float, depth_m: float) -> float:
    return surface_pressure_for_depth_sample(pressure_pa=pressure_pa, depth_m=depth_m)
