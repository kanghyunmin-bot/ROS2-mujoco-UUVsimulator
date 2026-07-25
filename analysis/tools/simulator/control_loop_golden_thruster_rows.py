"""Row filtering helpers for golden control-loop thruster summaries."""

from __future__ import annotations

from control_loop_golden_math import finite_float


def rows_with_wall_time(thruster_rows: list[dict[str, str]]) -> list[tuple[float, dict[str, str]]]:
    row_wall: list[tuple[float, dict[str, str]]] = []
    for row in thruster_rows:
        wall = finite_float(row.get("wall_mono_s"))
        if wall is not None:
            row_wall.append((wall, row))
    return row_wall


def rows_in_window(
    row_wall: list[tuple[float, dict[str, str]]],
    *,
    start: float,
    end: float,
) -> list[dict[str, str]]:
    return [row for wall, row in row_wall if start <= wall <= end]


__all__ = ["rows_in_window", "rows_with_wall_time"]
