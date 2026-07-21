"""Shared GUI process helpers for simulator and ROS processes."""

from __future__ import annotations

import subprocess

from .process_env import arg_present, default_sim_stack_backend, env_flag, sim_stack_backend
from .process_scan import matching_process_commands
from .process_termination import terminate_process_group


class ProcessCommonMixin:
    @staticmethod
    def _env_flag(name: str, default: bool = False) -> bool:
        return env_flag(name, default)

    @staticmethod
    def _arg_present(args: list[str], option: str) -> bool:
        return arg_present(args, option)

    @staticmethod
    def _default_sim_stack_backend() -> str:
        return default_sim_stack_backend()

    def _sim_stack_backend(self) -> str:
        return sim_stack_backend()

    @staticmethod
    def _matching_process_commands(patterns: tuple[str, ...]) -> list[str]:
        return matching_process_commands(patterns)

    @staticmethod
    def _terminate_process_group(proc: subprocess.Popen[str] | None, timeout_s: float = 4.0) -> None:
        terminate_process_group(proc, timeout_s)


__all__ = ["ProcessCommonMixin"]
