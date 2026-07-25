"""RC override fields for real-start state payloads."""

from __future__ import annotations

from typing import Any

from real_start_common import finite


def rc_override_ch1_8_from_row(row: dict[str, Any]) -> list[int]:
    rc = []
    for idx in range(1, 9):
        value = finite(row.get(f"rc_override_ch{idx}"), 1500.0)
        rc.append(int(round(value if value > 0.0 else 1500.0)))
    return rc


__all__ = ["rc_override_ch1_8_from_row"]
