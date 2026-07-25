"""start_sitl log parser for enforced parameter contract audits."""

from __future__ import annotations

from pathlib import Path

from audit_closed_loop_param_lines import (
    is_start_sitl_enforced_header,
    is_start_sitl_enforced_row,
    parse_param_assignment,
)


def parse_start_sitl_enforced_params(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    if not path.exists():
        return params
    in_block = False
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if is_start_sitl_enforced_header(raw_line):
            in_block = True
            continue
        if not in_block:
            continue
        if not is_start_sitl_enforced_row(raw_line):
            break
        assignment = parse_param_assignment(raw_line)
        if assignment is not None:
            params[assignment[0]] = assignment[1]
    return params


__all__ = ["parse_start_sitl_enforced_params"]
