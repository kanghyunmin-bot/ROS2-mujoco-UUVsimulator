"""MuJoCo id lookup helpers for the Ping360 simulator."""

from __future__ import annotations

import mujoco


def lookup_ping360_model_ids(model: mujoco.MjModel, site_name: str) -> tuple[int, int]:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return int(site_id), int(base_body_id)


__all__ = ["lookup_ping360_model_ids"]
