"""MuJoCo actuator-site lookup helpers for actuator wrench audits."""

from __future__ import annotations


def actuator_site_id(model: mujoco.MjModel, name: str) -> int:
    import mujoco

    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid >= 0:
        try:
            if int(model.actuator_trntype[aid]) == int(mujoco.mjtTrn.mjTRN_SITE):
                sid = int(model.actuator_trnid[aid, 0])
                if sid >= 0:
                    return sid
        except Exception:
            pass
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")


__all__ = ["actuator_site_id"]
