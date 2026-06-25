"""Run-mode and EKF-contract normalization for GUI-started simulator runs."""

from __future__ import annotations


def normalize_run_mode(raw: str) -> str:
    run_mode = str(raw).strip().lower()
    if run_mode not in {"closed_loop", "plant_replay"}:
        return "closed_loop"
    return run_mode


def normalize_ekf_contract(raw: str) -> str:
    ekf_contract = str(raw).strip().lower()
    if ekf_contract in {
        "poshold_extnav",
        "poshold_extnav_412",
        "real_param_parity",
        "real-ekf",
        "real_ekf",
        "extnav",
    }:
        return "poshold_extnav"
    return "althold_baro"


__all__ = ["normalize_run_mode", "normalize_ekf_contract"]
