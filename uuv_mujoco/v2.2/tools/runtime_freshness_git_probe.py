"""Small git probes used by the active-runtime freshness check."""

from __future__ import annotations

import subprocess
from pathlib import Path


def run_git(workspace: Path, args: list[str], *, timeout_s: float = 8.0) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=str(workspace),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            check=False,
        )
    except Exception as exc:
        return 127, str(exc)
    return completed.returncode, completed.stdout.rstrip()


def git_text(workspace: Path, args: list[str]) -> str:
    code, output = run_git(workspace, args)
    return output if code == 0 else ""
