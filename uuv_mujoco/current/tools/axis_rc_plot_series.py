"""Series helpers for axis RC response plots."""

from __future__ import annotations

import math
from typing import Any


def values(samples: list[dict[str, Any]], key: str) -> list[float]:
    return [float(sample.get(key, float("nan"))) for sample in samples]


def relative_time(samples: list[dict[str, Any]]) -> list[float]:
    if not samples:
        return []
    t0 = float(samples[0]["t"])
    return [float(sample["t"]) - t0 for sample in samples]


def dvl_speed_series(samples: list[dict[str, Any]]) -> tuple[list[float], list[float], list[float], list[float]]:
    dvl_vx = values(samples, "dvl_vx")
    dvl_vy = values(samples, "dvl_vy")
    dvl_vz = values(samples, "dvl_vz")
    dvl_speed = [
        math.sqrt(vx * vx + vy * vy + vz * vz)
        if math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)
        else float("nan")
        for vx, vy, vz in zip(dvl_vx, dvl_vy, dvl_vz)
    ]
    return dvl_vx, dvl_vy, dvl_vz, dvl_speed


__all__ = ["dvl_speed_series", "relative_time", "values"]
