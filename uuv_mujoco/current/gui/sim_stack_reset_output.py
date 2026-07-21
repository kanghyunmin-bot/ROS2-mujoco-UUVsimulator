"""Output parsing helpers for simulator reset logs."""

from __future__ import annotations

from typing import Iterable


def shorten_reset_line(line: str, *, max_len: int = 150) -> str:
    return line if len(line) <= max_len else f"{line[: max_len - 3]}..."


def iter_reset_status_lines(stdout: Iterable[str] | None) -> Iterable[str]:
    if stdout is None:
        return
    for raw_line in stdout:
        line = str(raw_line).strip()
        if line.startswith("[reset]"):
            yield shorten_reset_line(line)


__all__ = ["iter_reset_status_lines", "shorten_reset_line"]
