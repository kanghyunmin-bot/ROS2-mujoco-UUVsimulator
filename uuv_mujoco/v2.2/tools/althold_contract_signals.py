"""Signal assembly for ALT_HOLD DataFlash contract analysis."""

from __future__ import annotations

from typing import Any

import numpy as np

from althold_contract_bin import interp, mode_at, vertical_plant_cmd_from_rcou
from althold_contract_model import rc3_to_expected_althold_climb


def build_signals(streams: dict[str, dict[str, np.ndarray]], modes: list[dict]) -> dict[str, Any]:
    t = streams["RCIN"]["t"]
    rc3 = streams["RCIN"]["C3"]
    rcou = {key: interp(streams, "RCOU", key, t) for key in ["C5", "C6", "C7", "C8"]}
    return {
        "t": t,
        "rc3": rc3,
        "expected": np.asarray([rc3_to_expected_althold_climb(v) for v in rc3], dtype=float),
        "dcrt": interp(streams, "CTUN", "DCRt", t),
        "pscd_tvd": interp(streams, "PSCD", "TVD", t),
        "pscd_vd": interp(streams, "PSCD", "VD", t),
        "sim_pd": interp(streams, "SIM2", "PD", t),
        "sim_vd": interp(streams, "SIM2", "VD", t),
        "visv_vz": interp(streams, "VISV", "VZ", t),
        "rcou": rcou,
        "plant_cmd": vertical_plant_cmd_from_rcou(rcou["C5"], rcou["C6"], rcou["C7"], rcou["C8"]),
        "att": {key: interp(streams, "ATT", key, t) for key in ["Roll", "Pitch"]},
        "rate": {key: interp(streams, "RATE", key, t) for key in ["ROut", "POut"]},
        "mode_names": mode_at(t, modes),
    }


__all__ = ["build_signals"]
