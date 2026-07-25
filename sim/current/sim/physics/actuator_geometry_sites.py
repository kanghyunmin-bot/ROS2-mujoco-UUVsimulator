"""Actuator-to-site lookup helpers."""

from __future__ import annotations

from typing import Any, Mapping, Sequence


def actuator_site_id(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    name: str,
) -> int:
    actuator_id = int(actuator_ids.get(name, -1))
    if actuator_id >= 0:
        try:
            if int(model.actuator_trntype[actuator_id]) == int(mujoco_module.mjtTrn.mjTRN_SITE):
                site_id = int(model.actuator_trnid[actuator_id, 0])
                if site_id >= 0:
                    return site_id
        except Exception:
            pass
    return int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_SITE, f"thr_{name}"))


def actuator_site_ids(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    names: Sequence[str],
) -> dict[str, int]:
    return {
        name: actuator_site_id(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            name=name,
        )
        for name in names
    }


__all__ = ["actuator_site_id", "actuator_site_ids"]
