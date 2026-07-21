"""Scene builder setup for viewer debug drawing."""

from __future__ import annotations

from typing import Any

from sim.runtime.viewer_loop_draw_math import normalize_vector
from sim.runtime.viewer_scene import ViewerSceneBuilder


def prepare_viewer_scene(runtime: Any, viewer: Any) -> tuple[Any, Any]:
    runtime.viewer_controls.apply_camera(
        viewer=viewer,
        mujoco_module=runtime.mujoco,
        base_id=runtime.base_id,
        camera_ids=runtime.camera_ids,
    )
    scene = ViewerSceneBuilder(
        user_scn=viewer.user_scn,
        mujoco=runtime.mujoco,
        normalize=normalize_vector,
        sim_time=float(runtime.data.time),
        thrust_force_max=float(runtime.thruster_force_max),
    )
    base_rot = runtime.data.xmat[runtime.base_id].reshape(3, 3)
    return scene, base_rot


__all__ = ["prepare_viewer_scene"]
