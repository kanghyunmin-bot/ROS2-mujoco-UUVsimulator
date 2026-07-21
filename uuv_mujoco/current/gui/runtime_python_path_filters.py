"""Path predicates used before ROS GUI imports."""

from __future__ import annotations

import os


def is_under_prefix(path: str, prefix: str) -> bool:
    try:
        return os.path.commonpath([prefix, os.path.realpath(path)]) == prefix
    except ValueError:
        return False


def is_foreign_env_path(path: str, current_prefix: str) -> bool:
    real_path = os.path.realpath(path)
    if "/miniconda3/envs/" in real_path or "/.venvs/" in real_path:
        return not is_under_prefix(real_path, current_prefix)
    return False


def should_drop_import_entry(entry: str, *, current_prefix: str, user_site: str | None) -> bool:
    if user_site and os.path.realpath(entry) == user_site:
        return True
    return is_foreign_env_path(entry, current_prefix)


def filtered_pythonpath_entries(pythonpath: str | None, current_prefix: str) -> list[str]:
    if not pythonpath:
        return []
    return [
        entry
        for entry in pythonpath.split(os.pathsep)
        if entry and not is_foreign_env_path(entry, current_prefix)
    ]


__all__ = [
    "filtered_pythonpath_entries",
    "is_foreign_env_path",
    "is_under_prefix",
    "should_drop_import_entry",
]
