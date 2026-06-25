"""Optional overlay drawing for the MuJoCo viewer loop."""

from __future__ import annotations

from typing import Any

from sim.runtime.viewer_scene import draw_net_and_buoyancy_debug, draw_sensor_markers, draw_thruster_debug


def draw_optional_viewer_overlays(runtime: Any, scene: Any, *, base_rot) -> None:
    if runtime.viewer_controls.show_debug or runtime.viewer_controls.show_thruster_labels:
        draw_thruster_debug(
            scene,
            data=runtime.data,
            model=runtime.model,
            act=runtime.act,
            base_rot=base_rot,
            thruster_site_ids=runtime.thruster_site_ids,
            thruster_names=runtime.thruster_names,
            show_debug=runtime.viewer_controls.show_debug,
            show_labels=runtime.viewer_controls.show_thruster_labels,
        )

    if runtime.viewer_controls.show_sensor_overlay:
        draw_sensor_markers(scene, data=runtime.data, sensor_site_ids=runtime.sensor_site_ids)

    if runtime.viewer_controls.show_debug:
        draw_net_and_buoyancy_debug(
            scene,
            data=runtime.data,
            model=runtime.model,
            act=runtime.act,
            base_rot=base_rot,
            com=runtime.data.xipos[runtime.base_id].copy(),
            thruster_names=runtime.thruster_names,
            last_buoy_force=runtime.get_last_buoy_force(),
            last_buoy_point=runtime.get_last_buoy_point(),
        )


__all__ = ["draw_optional_viewer_overlays"]
