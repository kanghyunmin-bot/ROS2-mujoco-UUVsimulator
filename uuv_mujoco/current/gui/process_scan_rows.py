"""Process-list row helpers for GUI process scanners."""

from __future__ import annotations

import subprocess


def process_listing_text() -> str:
    try:
        result = subprocess.run(
            ["ps", "-axo", "pid=,command="],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1.0,
            check=False,
        )
    except Exception:
        return ""
    if result.returncode != 0:
        return ""
    return str(result.stdout)


def parse_process_line(raw_line: str) -> tuple[int, str] | None:
    line = raw_line.strip()
    if not line:
        return None
    try:
        pid_text, command = line.split(maxsplit=1)
        return int(pid_text), command
    except ValueError:
        return None


__all__ = ["parse_process_line", "process_listing_text"]
