"""Complexity scoring for the refactor inventory tool."""

from __future__ import annotations


def compute_complexity_score(
    *,
    loc: int,
    branches: int,
    functions: int,
    classes: int,
    largest_symbol_loc: int,
    parse_error: bool = False,
) -> int:
    """Rank refactor hotspots by structure instead of raw file length."""
    if parse_error:
        return int(loc) + 100
    if branches == 0 and functions == 0 and classes == 0:
        return min(10, max(1, int(loc) // 20))
    return (
        int(largest_symbol_loc)
        + int(branches) * 8
        + int(functions) * 2
        + int(classes) * 3
        + min(10, int(loc) // 50)
    )


__all__ = ["compute_complexity_score"]
