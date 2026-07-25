"""Initial hold release velocity helpers."""

from __future__ import annotations

import numpy as np


def reset_release_state(*, data, mujoco, model, world_qvel_adr: int) -> None:
    """Clear free-joint velocity before release velocity is applied."""
    data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
    data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)


def apply_release_velocity(
    *,
    data,
    mujoco,
    model,
    base_id: int,
    world_qvel_adr: int,
    velocity_body: np.ndarray | None,
    angular_velocity_body: np.ndarray | None,
) -> None:
    """Apply captured body-frame release velocities to MuJoCo qvel."""
    base_rot = data.xmat[base_id].reshape(3, 3)
    applied: list[str] = []
    _apply_linear_velocity(data, world_qvel_adr, base_rot, velocity_body, applied)
    _apply_angular_velocity(data, world_qvel_adr, angular_velocity_body, applied)
    if not applied:
        return
    data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
    mujoco.mj_forward(model, data)
    _print_release_velocity_check(data, base_id=base_id, world_qvel_adr=world_qvel_adr, applied=applied)


def _apply_linear_velocity(
    data,
    world_qvel_adr: int,
    base_rot: np.ndarray,
    velocity_body: np.ndarray | None,
    applied: list[str],
) -> None:
    if velocity_body is None:
        return
    velocity_body = np.asarray(velocity_body, dtype=np.float64)
    if velocity_body.shape != (3,) or not np.all(np.isfinite(velocity_body)):
        return
    data.qvel[world_qvel_adr : world_qvel_adr + 3] = base_rot @ velocity_body
    applied.append("linear=" + np.array2string(velocity_body, precision=4) + " m/s")


def _apply_angular_velocity(
    data,
    world_qvel_adr: int,
    angular_velocity_body: np.ndarray | None,
    applied: list[str],
) -> None:
    if angular_velocity_body is None:
        return
    angular_velocity_body = np.asarray(angular_velocity_body, dtype=np.float64)
    if angular_velocity_body.shape != (3,) or not np.all(np.isfinite(angular_velocity_body)):
        return
    data.qvel[world_qvel_adr + 3 : world_qvel_adr + 6] = angular_velocity_body
    applied.append("angular=" + np.array2string(angular_velocity_body, precision=4) + " rad/s")


def _print_release_velocity_check(data, *, base_id: int, world_qvel_adr: int, applied: list[str]) -> None:
    print("[runtime] release velocity state set: " + ", ".join(applied), flush=True)
    base_rot_after = data.xmat[base_id].reshape(3, 3)
    linear_body_check = base_rot_after.T @ data.qvel[world_qvel_adr : world_qvel_adr + 3]
    angular_body_check = data.qvel[world_qvel_adr + 3 : world_qvel_adr + 6].copy()
    print(
        "[runtime] release velocity qvel check: "
        f"linear_body={np.array2string(linear_body_check, precision=4)} m/s, "
        f"angular_body={np.array2string(angular_body_check, precision=4)} rad/s",
        flush=True,
    )


__all__ = ["apply_release_velocity", "reset_release_state"]
