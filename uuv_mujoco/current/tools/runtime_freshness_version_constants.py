"""Constants for active-runtime version metadata."""

from __future__ import annotations


DEFAULT_FRESHNESS_POLICY = "origin/main is the latest source baseline; validate changes before publishing."

PRIMARY_RUNNER = "uuv_mujoco/current/run_uuv_mujoco.py"
COMPATIBILITY_RUNNER = "uuv_mujoco/current/run_urdf_full.py"
SOURCE_AUDIT_CHECK = "active_runtime_alias_current"

RUNTIME_VERSION_NOTE = (
    "v2.2 is a compatibility directory name. New launch, GUI, setup, and "
    "validation paths must resolve through uuv_mujoco/current."
)

ROOT_LAUNCHERS = {
    "mujoco": "uuv_mujoco/run_mujoco.sh",
    "sitl_mujoco": "uuv_mujoco/start_sitl_mujoco.sh",
    "docker_sitl_mujoco": "uuv_mujoco/start_docker_sitl_mujoco.sh",
    "reset": "uuv_mujoco/reset_sim.sh",
}


__all__ = [
    "COMPATIBILITY_RUNNER",
    "DEFAULT_FRESHNESS_POLICY",
    "PRIMARY_RUNNER",
    "ROOT_LAUNCHERS",
    "RUNTIME_VERSION_NOTE",
    "SOURCE_AUDIT_CHECK",
]
