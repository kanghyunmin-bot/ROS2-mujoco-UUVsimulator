"""Constants for active-runtime version metadata."""

from __future__ import annotations


DEFAULT_FRESHNESS_POLICY = (
    "origin/uuv_sim is the source branch for this active runtime; origin/main and "
    "origin/master are different layout branches and must not be merged blindly into "
    "the dirty simulator workspace."
)

PRIMARY_RUNNER = "sim/current/run_uuv_mujoco.py"
COMPATIBILITY_RUNNER = ""
SOURCE_AUDIT_CHECK = "active_runtime_alias_current"

RUNTIME_VERSION_NOTE = "The active simulator runtime is the concrete sim/current directory."

ROOT_LAUNCHERS = {
    "mujoco": "sim/run_mujoco.sh",
    "sitl_mujoco": "sim/start_sitl_mujoco.sh",
    "reset": "sim/reset_sim.sh",
}


__all__ = [
    "COMPATIBILITY_RUNNER",
    "DEFAULT_FRESHNESS_POLICY",
    "PRIMARY_RUNNER",
    "ROOT_LAUNCHERS",
    "RUNTIME_VERSION_NOTE",
    "SOURCE_AUDIT_CHECK",
]
