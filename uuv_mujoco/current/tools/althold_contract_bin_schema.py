"""DataFlash stream schema for ALT_HOLD contract extraction."""

from __future__ import annotations


BIN_STREAM_FIELDS: dict[str, list[str]] = {
    "RCIN": ["C3"],
    "RCOU": ["C5", "C6", "C7", "C8"],
    "CTUN": ["DAlt", "Alt", "DCRt", "CRt"],
    "PSCD": ["TPD", "PD", "TVD", "VD"],
    "ATT": ["Roll", "Pitch", "DesRoll", "DesPitch"],
    "RATE": ["ROut", "POut", "YOut", "AOut"],
    "VISV": ["VZ", "Ign"],
    "SIM2": ["PD", "VD"],
    "BARO": ["I", "Alt", "Press", "Health"],
}


__all__ = ["BIN_STREAM_FIELDS"]
