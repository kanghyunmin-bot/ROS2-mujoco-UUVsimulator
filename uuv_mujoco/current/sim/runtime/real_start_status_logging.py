"""One-shot runtime logs for real-start status payloads."""

from __future__ import annotations


def real_start_status_is_ok(payload: dict[str, object]) -> bool:
    return bool(payload.get("required") and payload.get("ok"))


def real_start_status_is_released(payload: dict[str, object]) -> bool:
    return bool(real_start_status_is_ok(payload) and payload.get("released"))


def log_real_start_ok(payload: dict[str, object]) -> None:
    print(
        "[runtime] real start state OK: "
        f"{payload.get('depth_contract', 'depth')}_depth_error="
        f"{float(payload.get('depth_error_m', 0.0)):+.4f}m "
        f"pressure_error={float(payload.get('pressure_error_pa', 0.0)):+.2f}Pa "
        f"xy_error={float(payload.get('xy_error_m', 0.0)):.4f}m "
        f"attitude_error={float(payload.get('attitude_error_rad', 0.0)):.4f}rad "
        f"velocity_error={float(payload.get('velocity_error_mps', 0.0)):.4f}m/s",
        flush=True,
    )


def log_real_start_released() -> None:
    print("[runtime] real start state released after OK", flush=True)


__all__ = [
    "log_real_start_ok",
    "log_real_start_released",
    "real_start_status_is_ok",
    "real_start_status_is_released",
]
