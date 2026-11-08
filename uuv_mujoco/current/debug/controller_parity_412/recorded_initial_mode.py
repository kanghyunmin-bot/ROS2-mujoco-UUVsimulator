"""Recorded-mode initialization for sensor replay."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import re


def recorded_mode_id(name: str, mode_ids: Mapping[str, int]) -> int:
    """Resolve a known mode name or MAVROS's equivalent numeric fallback."""
    if name in mode_ids:
        return mode_ids[name]
    match = re.fullmatch(r"CMODE\((\d+)\)", name)
    if match and int(match[1]) in mode_ids.values():
        return int(match[1])
    raise ValueError(f"Unsupported recorded initial mode: {name}")


def select_initial_mode_row(states: Sequence[Sequence], mode_ids: Mapping[str, int]) -> int:
    """Select a recorded initial-mode row, whose first field is receipt time [s]."""
    for index, row in enumerate(states):
        if not bool(row[1]) or not row[3]:
            continue
        recorded_mode_id(row[3], mode_ids)
        return index
    raise ValueError("No connected recorded state has an initial mode")
