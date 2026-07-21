"""Signal math helpers for ALT_HOLD contract analysis."""

from __future__ import annotations

import math

import numpy as np

from althold_contract_model import RC_NEUTRAL


def interp(streams: dict[str, dict[str, np.ndarray]], stream: str, field: str, t: np.ndarray) -> np.ndarray:
    if stream not in streams or field not in streams[stream] or len(streams[stream]["t"]) == 0:
        return np.full_like(t, math.nan, dtype=float)
    st = streams[stream]["t"]
    sv = streams[stream][field]
    return np.interp(t, st, sv, left=math.nan, right=math.nan)


def mode_at(times: np.ndarray, modes: list[dict]) -> list[str]:
    if not modes:
        return ["unknown"] * len(times)
    idx = 0
    names = []
    for t in times:
        while idx + 1 < len(modes) and modes[idx + 1]["t"] <= t:
            idx += 1
        names.append(str(modes[idx]["name"]))
    return names


def vertical_plant_cmd_from_rcou(c5: np.ndarray, c6: np.ndarray, c7: np.ndarray, c8: np.ndarray) -> np.ndarray:
    # Down-positive MuJoCo plant command. Negative values correspond to upward thrust.
    signs = np.asarray([-1.0, 1.0, 1.0, -1.0])
    stacked = np.vstack([c5, c6, c7, c8])
    return np.nanmean(((stacked - RC_NEUTRAL) / 400.0) * signs[:, None], axis=0)


__all__ = ["interp", "mode_at", "vertical_plant_cmd_from_rcou"]
