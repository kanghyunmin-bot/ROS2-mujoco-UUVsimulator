"""Reset-script path helpers for physics-profile restarts."""

from __future__ import annotations


def sim_stack_reset_paths():
    from .config import RESET_SIM_STACK_SCRIPT, SIM_STACK_DIR

    return RESET_SIM_STACK_SCRIPT, SIM_STACK_DIR


def reset_script_missing_status(reset_script) -> str:
    return f"reset script missing: {reset_script}"


__all__ = ["reset_script_missing_status", "sim_stack_reset_paths"]
