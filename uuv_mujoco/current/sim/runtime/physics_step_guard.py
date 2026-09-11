"""Stop a numerically failed step before an automatic reset reaches the controller."""

from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def step_with_reset_guard(mujoco, model, data) -> None:
    """Advance physics, saving the preceding state if MuJoCo resets or diverges."""
    previous_time = float(data.time)
    # Save a compact state, not the mesh/model or the full contact workspace.
    previous = {
        name: np.array(getattr(data, name), copy=True)
        for name in (
            "qpos",
            "qvel",
            "qacc",
            "qacc_warmstart",
            "ctrl",
            "eq_active",
            "xfrc_applied",
            "qfrc_applied",
        )
    }
    warning_ids = [
        int(mujoco.mjtWarning.mjWARN_BADQPOS),
        int(mujoco.mjtWarning.mjWARN_BADQVEL),
        int(mujoco.mjtWarning.mjWARN_BADQACC),
    ]
    old_counts = [int(data.warning[i].number) for i in warning_ids]
    mujoco.mj_step(model, data)
    failed = float(data.time) < previous_time or any(
        int(data.warning[i].number) > old for i, old in zip(warning_ids, old_counts)
    )
    failed |= not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    if not failed:
        return
    folder = Path(__file__).resolve().parents[2] / "generated" / "physics_failures"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%f") + ".npz")
    np.savez_compressed(
        path,
        time=previous_time,
        timestep=float(model.opt.timestep),
        integrator=int(model.opt.integrator),
        dof_armature=np.array(model.dof_armature),
        dof_damping=np.array(model.dof_damping),
        dof_joint_names=np.asarray([model.joint(int(j)).name for j in model.dof_jntid]),
        **previous
    )
    raise RuntimeError(
        f"Physics instability at {previous_time:.6f} s; stopped instead of continuing "
        f"after an automatic reset. Previous state: {path}"
    )
