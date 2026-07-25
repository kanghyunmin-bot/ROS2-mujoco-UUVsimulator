"""Process-list scanners shared by GUI process controls."""

from __future__ import annotations

import os
import re

from .process_scan_rows import parse_process_line, process_listing_text


def matching_process_commands(patterns: tuple[str, ...]) -> list[str]:
    compiled = [re.compile(pattern) for pattern in patterns]
    matches: list[str] = []
    own_pid = os.getpid()
    for raw_line in process_listing_text().splitlines():
        parsed = parse_process_line(raw_line)
        if parsed is None:
            continue
        pid, command = parsed
        if pid == own_pid:
            continue
        if any(regex.search(command) for regex in compiled):
            matches.append(command)
    return matches


__all__ = ["matching_process_commands"]
