"""Working-tree dirty path parsing for active-runtime freshness evidence."""

from __future__ import annotations


ACTIVE_RUNTIME_PREFIXES = ("uuv_mujoco/v2.2/", "uuv_mujoco/current/")
ACTIVE_RUNTIME_FILES = {
    "uuv_mujoco/current",
    "uuv_mujoco/CURRENT.md",
    "uuv_mujoco/RUNTIME_VERSION.json",
}


def porcelain_lines(status_text: str) -> list[str]:
    return [line for line in status_text.splitlines() if line.strip()]


def porcelain_path(line: str) -> str:
    path = line[3:].strip() if len(line) > 3 else line.strip()
    if " -> " in path:
        path = path.rsplit(" -> ", 1)[-1].strip()
    return path


def dirty_paths(status_text: str) -> list[str]:
    return [porcelain_path(line) for line in porcelain_lines(status_text)]


def active_runtime_dirty_paths(paths: list[str]) -> list[str]:
    return [
        path
        for path in paths
        if path.startswith(ACTIVE_RUNTIME_PREFIXES) or path in ACTIVE_RUNTIME_FILES
    ]
