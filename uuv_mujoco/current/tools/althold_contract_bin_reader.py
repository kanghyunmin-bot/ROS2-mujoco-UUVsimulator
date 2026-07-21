"""DataFlash BIN reader for ALT_HOLD contract analysis."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from althold_contract_bin_message_loop import collect_streams_from_bin


def read_bin(path: Path) -> tuple[dict[str, dict[str, np.ndarray]], list[dict], dict[str, float]]:
    streams, modes, params = collect_streams_from_bin(path)
    if not streams:
        raise SystemExit(f"no usable streams in {path}")
    first_t = _first_time_us_s(streams, modes)
    return _streams_to_arrays(streams, first_t), _normalize_modes(modes, first_t), params


def _first_time_us_s(streams: dict[str, dict[str, list[float]]], modes: list[dict]) -> float:
    first_t = min(values["t"][0] for values in streams.values() if values["t"])
    if modes:
        first_t = min(first_t, modes[0]["t"])
    return float(first_t)


def _streams_to_arrays(
    streams: dict[str, dict[str, list[float]]],
    first_t: float,
) -> dict[str, dict[str, np.ndarray]]:
    arrays: dict[str, dict[str, np.ndarray]] = {}
    for name, values in streams.items():
        arrays[name] = {}
        for field, vals in values.items():
            arr = np.asarray(vals, dtype=float)
            if field == "t":
                arr = arr - first_t
            arrays[name][field] = arr
    return arrays


def _normalize_modes(modes: list[dict], first_t: float) -> list[dict]:
    for mode in modes:
        mode["t"] = mode["t"] - first_t
    return modes


__all__ = ["read_bin"]
