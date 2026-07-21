"""Process command helpers for simulator stack reset."""

from __future__ import annotations

import subprocess

from .config import RESET_SIM_STACK_SCRIPT, SIM_STACK_DIR, STOP_DOCKER_SITL_SCRIPT


def reset_script_exists() -> bool:
    return RESET_SIM_STACK_SCRIPT.exists()


def run_script_blocking(script_path, args: list[str], *, timeout_s: float) -> None:
    if not script_path.exists():
        return
    try:
        subprocess.run(
            [str(script_path), *args],
            cwd=str(SIM_STACK_DIR),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=float(timeout_s),
            check=False,
            start_new_session=True,
        )
    except Exception:
        pass


def reset_sim_stack_blocking(timeout_s: float = 12.0) -> None:
    run_script_blocking(RESET_SIM_STACK_SCRIPT, ["--wipe-eeprom"], timeout_s=timeout_s)


def stop_docker_sitl_blocking(timeout_s: float = 12.0) -> None:
    run_script_blocking(STOP_DOCKER_SITL_SCRIPT, [], timeout_s=timeout_s)


def open_reset_sim_stack_process():
    return subprocess.Popen(
        [str(RESET_SIM_STACK_SCRIPT), "--wipe-eeprom"],
        cwd=str(SIM_STACK_DIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )


__all__ = [
    "open_reset_sim_stack_process",
    "reset_script_exists",
    "reset_sim_stack_blocking",
    "run_script_blocking",
    "stop_docker_sitl_blocking",
]
