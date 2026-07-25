"""Parameter file parsing helpers for closed-loop contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_closed_loop_param_lines import parse_param_assignment
from audit_closed_loop_param_start_sitl import parse_start_sitl_enforced_params


def parse_param_file(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    if not path.exists():
        return params
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        assignment = parse_param_assignment(raw_line)
        if assignment is not None:
            params[assignment[0]] = assignment[1]
    return params
