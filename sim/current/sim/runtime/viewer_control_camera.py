"""Camera application helpers for MuJoCo viewer controls."""

from __future__ import annotations

from typing import Any, Mapping


def apply_viewer_camera(
    state: Any,
    *,
    viewer: Any,
    mujoco_module: Any,
    base_id: int,
    camera_ids: Mapping[str, int],
) -> None:
    """Apply the selected camera mode to the MuJoCo viewer camera."""
    if state.camera_mode in camera_ids:
        cam = viewer.cam
        cam.type = int(mujoco_module.mjtCamera.mjCAMERA_FIXED)
        cam.fixedcamid = int(camera_ids[state.camera_mode])
        cam.trackbodyid = -1
        return
    if state.follow_camera_enabled:
        apply_tracking_camera(state, viewer=viewer, mujoco_module=mujoco_module, base_id=base_id)
        return
    release_fixed_or_tracking_camera(viewer=viewer, mujoco_module=mujoco_module)


def apply_tracking_camera(state: Any, *, viewer: Any, mujoco_module: Any, base_id: int) -> None:
    cam = viewer.cam
    cam.type = int(mujoco_module.mjtCamera.mjCAMERA_TRACKING)
    cam.trackbodyid = int(base_id)
    if not state.follow_camera_initialized:
        cam.distance = state.follow_distance
        cam.elevation = state.follow_elevation
        cam.azimuth = state.follow_azimuth
        state.follow_camera_initialized = True


def release_fixed_or_tracking_camera(*, viewer: Any, mujoco_module: Any) -> None:
    if int(viewer.cam.type) not in (
        int(mujoco_module.mjtCamera.mjCAMERA_TRACKING),
        int(mujoco_module.mjtCamera.mjCAMERA_FIXED),
    ):
        return
    viewer.cam.type = int(mujoco_module.mjtCamera.mjCAMERA_FREE)
    viewer.cam.fixedcamid = -1
    viewer.cam.trackbodyid = -1


__all__ = ["apply_tracking_camera", "apply_viewer_camera", "release_fixed_or_tracking_camera"]
