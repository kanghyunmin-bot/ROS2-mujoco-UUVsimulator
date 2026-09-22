"""Count finalized demonstration manifests without loading images or samples."""
import json
from pathlib import Path

DETACH_TASKS = {
    "Approach the yellow buoy, align the fixed fork, and detach the buoy.",
    "Move toward the yellow buoy, align the fixed fork, and release the buoy from its attachment.",
    "Use the fixed fork to detach the yellow buoy after approaching and aligning with it.",
}


def collection_counts(root: Path) -> dict:
    counts = dict(total=0, success=0, failure=0, other=0, unreadable=0, goal=50)
    for path in root.glob("*/staging/episode_*/manifest.json"):
        try:
            manifest = json.loads(path.read_text())
            if not isinstance(manifest, dict):
                raise ValueError("Invalid manifest")
            provenance = manifest.get("provenance") or {}
            if not isinstance(provenance, dict):
                raise ValueError("Invalid provenance")
        except (OSError, ValueError):
            counts["unreadable"] += 1
            continue
        counts["total"] += 1
        is_task = (isinstance(manifest.get("task"), str)
                   and manifest["task"] in DETACH_TASKS
                   and provenance.get("collection_kind") == "task_demonstration"
                   and provenance.get("data_source") == "simulation")
        if is_task and manifest.get("success") is True:
            counts["success"] += 1
        elif is_task and manifest.get("success") is False:
            counts["failure"] += 1
        else:
            counts["other"] += 1
    return counts
