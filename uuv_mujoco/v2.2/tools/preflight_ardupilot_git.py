"""Read-only git status collection for ArduPilot preflight checks."""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any

WATCHED_PATHS = (
    "ArduSub/control_althold.cpp",
    "libraries/AP_Common/missing/fenv.h",
)


def run_git(ardupilot_dir: Path, args: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(
        ["git", "-C", str(ardupilot_dir), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    return proc.returncode, proc.stdout, proc.stderr


def collect_submodule_pointer(ardupilot_dir: Path) -> dict[str, Any]:
    workspace = ardupilot_dir.parent
    head_rc, head_out, head_err = run_git(ardupilot_dir, ["rev-parse", "HEAD"])
    index_rc, index_out, index_err = _run_git_at(workspace, ["ls-files", "-s", "--", ardupilot_dir.name])
    status_rc, status_out, status_err = _run_git_at(workspace, ["submodule", "status", "--", ardupilot_dir.name])
    recorded_sha = ""
    if index_rc == 0 and index_out.strip():
        parts = index_out.split()
        if len(parts) >= 2 and parts[0] == "160000":
            recorded_sha = parts[1]
    current_sha = head_out.strip() if head_rc == 0 else ""
    return {
        "workspace": str(workspace),
        "path": ardupilot_dir.name,
        "recorded_sha": recorded_sha,
        "current_sha": current_sha,
        "matches_recorded": bool(recorded_sha and current_sha and recorded_sha == current_sha),
        "head": {"returncode": head_rc, "stderr": head_err.strip()},
        "index": {"returncode": index_rc, "stderr": index_err.strip(), "raw": index_out.strip()},
        "submodule_status": {
            "returncode": status_rc,
            "stderr": status_err.strip(),
            "raw": status_out.strip(),
        },
    }


def _run_git_at(cwd: Path, args: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(
        ["git", "-C", str(cwd), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    return proc.returncode, proc.stdout, proc.stderr


def collect_status(ardupilot_dir: Path) -> dict[str, Any]:
    status_rc, status_out, status_err = run_git(ardupilot_dir, ["status", "--short"])
    diff_payload: dict[str, Any] = {}
    for rel_path in WATCHED_PATHS:
        rc, out, err = run_git(ardupilot_dir, ["diff", "--", rel_path])
        diff_payload[rel_path] = {
            "returncode": rc,
            "stderr": err.strip(),
            "dirty": bool(out.strip()),
            "diff": out,
        }
    return {
        "ardupilot_dir": str(ardupilot_dir),
        "submodule_pointer": collect_submodule_pointer(ardupilot_dir),
        "git_status": {
            "returncode": status_rc,
            "stderr": status_err.strip(),
            "short": status_out.splitlines(),
        },
        "watched_diffs": diff_payload,
    }


__all__ = ["WATCHED_PATHS", "collect_status", "collect_submodule_pointer", "run_git"]
