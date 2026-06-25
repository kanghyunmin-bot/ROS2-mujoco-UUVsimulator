"""Viewer debug geometry drawing for the MuJoCo runner."""

from __future__ import annotations

from typing import Any

from sim.runtime.viewer_loop_draw_math import normalize_vector
from sim.runtime.viewer_loop_draw_overlays import draw_optional_viewer_overlays
from sim.runtime.viewer_loop_scene_builder import prepare_viewer_scene


class ViewerLoopDrawMixin:
    def draw_scene(self, viewer: Any) -> None:
        with viewer.lock():
            user_scn = viewer.user_scn
            user_scn.ngeom = 0

            scene, base_rot = prepare_viewer_scene(self, viewer)
            draw_optional_viewer_overlays(self, scene, base_rot=base_rot)


__all__ = ["ViewerLoopDrawMixin", "normalize_vector"]
