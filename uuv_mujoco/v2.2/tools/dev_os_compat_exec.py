"""Command execution and executable-path helpers for dev OS checks."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Sequence


def run_command(argv: Sequence[str], timeout_s: float = 4.0) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            list(argv),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=float(timeout_s),
            check=False,
        )
    except FileNotFoundError:
        return 127, "not found"
    except subprocess.TimeoutExpired:
        return 124, "timeout"
    return int(completed.returncode), completed.stdout.strip()


def executable_path(raw: str | Path | None) -> Path | None:
    if raw is None:
        return None
    text = str(raw).strip()
    if not text:
        return None
    path = Path(text).expanduser()
    if path.is_file() and os.access(path, os.X_OK):
        # Preserve virtualenv launcher symlinks. Resolving .venv/bin/python to
        # /usr/bin/python discards Python's virtualenv prefix and site-packages.
        return path.absolute()
    return None


__all__ = ["executable_path", "run_command"]
