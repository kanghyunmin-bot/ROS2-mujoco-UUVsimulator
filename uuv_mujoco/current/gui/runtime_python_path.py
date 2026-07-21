"""Python import-path sanitization before ROS GUI imports."""

from __future__ import annotations

import os
import site
import sys

from .runtime_python_path_filters import filtered_pythonpath_entries, should_drop_import_entry


def sanitize_python_import_path() -> None:
    """Drop site-packages entries from other Python envs before importing ROS."""
    current_prefix = os.path.realpath(sys.prefix)

    pythonpath = os.environ.get("PYTHONPATH")
    if pythonpath:
        kept = filtered_pythonpath_entries(pythonpath, current_prefix)
        if kept:
            os.environ["PYTHONPATH"] = os.pathsep.join(kept)
        else:
            os.environ.pop("PYTHONPATH", None)

    try:
        user_site = os.path.realpath(site.getusersitepackages())
    except Exception:
        user_site = None

    filtered_sys_path = []
    for entry in sys.path:
        if should_drop_import_entry(entry, current_prefix=current_prefix, user_site=user_site):
            continue
        filtered_sys_path.append(entry)
    sys.path[:] = filtered_sys_path

    for entry in list(sys.path_importer_cache):
        if should_drop_import_entry(entry, current_prefix=current_prefix, user_site=user_site):
            sys.path_importer_cache.pop(entry, None)


__all__ = ["sanitize_python_import_path"]
