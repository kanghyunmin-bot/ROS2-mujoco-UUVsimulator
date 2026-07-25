"""Canvas drawing methods for attitude and depth widgets."""

from __future__ import annotations

from .control_attitude_draw import draw_attitude_canvas
from .control_depth_draw import draw_depth_canvas


class ControlDrawMixin:
    def _draw_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        draw_attitude_canvas(self.attitude_canvas, roll_deg, pitch_deg, yaw_deg)

    def _draw_depth(self, depth_m: float, source: str) -> None:
        draw_depth_canvas(self.depth_canvas, depth_m, source)
