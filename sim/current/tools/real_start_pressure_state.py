"""Bar30/AP_Baro pressure fields for real-start state payloads."""

from __future__ import annotations

import math
from typing import Any

from real_start_baro import (
    AP_BARO_FRONTEND_PA_PER_M,
    AP_BARO_SITL_GROUND_PRESSURE_PA,
    baro_frontend_depth_m,
    baro_json_depth_for_frontend_match,
    infer_baro_real_ground_pressure,
    surface_pressure_for_sample,
)
from real_start_common import finite


def build_pressure_state(
    *,
    rows: list[dict[str, Any]],
    time_col: str,
    row: dict[str, Any],
    depth_m: float,
    depth_source: str,
) -> dict[str, float | str]:
    static_pressure_pa = finite(row.get("static_pressure_pa"))
    baro_real_ground_pressure_pa, baro_real_ground_source = infer_baro_real_ground_pressure(rows, time_col)
    if not math.isfinite(baro_real_ground_pressure_pa) and math.isfinite(static_pressure_pa):
        baro_real_ground_pressure_pa = static_pressure_pa - AP_BARO_FRONTEND_PA_PER_M * max(0.0, depth_m)
        baro_real_ground_source = f"static_pressure_minus_{depth_source}"

    bar30_surface_pressure_pa = (
        surface_pressure_for_sample(static_pressure_pa, depth_m)
        if math.isfinite(static_pressure_pa)
        else math.nan
    )
    baro_json_depth_m = (
        baro_json_depth_for_frontend_match(static_pressure_pa, baro_real_ground_pressure_pa)
        if math.isfinite(static_pressure_pa) and math.isfinite(baro_real_ground_pressure_pa)
        else math.nan
    )
    baro_frontend_depth = (
        baro_frontend_depth_m(static_pressure_pa, baro_real_ground_pressure_pa)
        if math.isfinite(static_pressure_pa) and math.isfinite(baro_real_ground_pressure_pa)
        else math.nan
    )
    return {
        "static_pressure_pa": static_pressure_pa,
        "bar30_surface_pressure_pa": bar30_surface_pressure_pa,
        "baro_real_ground_pressure_pa": baro_real_ground_pressure_pa,
        "baro_real_ground_source": baro_real_ground_source,
        "baro_sitl_ground_pressure_pa": AP_BARO_SITL_GROUND_PRESSURE_PA,
        "baro_json_depth_m": baro_json_depth_m,
        "baro_frontend_depth_m": baro_frontend_depth,
    }


__all__ = ["build_pressure_state"]
