"""Headless/viewer loop dispatch helpers for the runtime entrypoint."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

import mujoco.viewer as mujoco_viewer

from sim.runtime.simulation_loop_runtime import ViewerRuntimeLoop, run_headless_loop


def run_headless_runtime(
    *,
    stop_event: Any,
    viewer_controls: Any,
    timestep: float,
    run_step: Callable[..., tuple[float, float, float, float]],
) -> None:
    print("[runtime] headless mode enabled: running without GLFW viewer", flush=True)
    run_headless_loop(
        stop_event=stop_event,
        viewer_controls=viewer_controls,
        timestep=float(timestep),
        run_step=run_step,
    )


def run_viewer_runtime(
    *,
    mujoco_module: Any,
    model: Any,
    data: Any,
    base_id: int,
    act: Mapping[str, int],
    camera_ids: Mapping[str, int],
    thruster_site_ids: Mapping[str, int],
    thruster_names: Sequence[str],
    sensor_site_ids: Mapping[str, int],
    viewer_controls: Any,
    stop_event: Any,
    run_step: Callable[..., tuple[float, float, float, float]],
    publish_ros_once: Callable[[], None],
    sensor_value: Callable[[str], Any],
    get_last_buoy_force: Callable[[], Any],
    get_last_buoy_point: Callable[[], Any],
    ros2_sensor_hz: float,
    viewer_fps: float,
    timestep: float,
    thruster_force_max: float,
    viewer_key_callback: Callable[[Any], None],
) -> None:
    with mujoco_viewer.launch_passive(model, data, key_callback=viewer_key_callback) as viewer:
        ViewerRuntimeLoop(
            mujoco=mujoco_module,
            model=model,
            data=data,
            base_id=base_id,
            act=act,
            camera_ids=camera_ids,
            thruster_site_ids=thruster_site_ids,
            thruster_names=thruster_names,
            sensor_site_ids=sensor_site_ids,
            viewer_controls=viewer_controls,
            stop_event=stop_event,
            run_step=run_step,
            publish_ros_once=publish_ros_once,
            sensor_value=sensor_value,
            get_last_buoy_force=get_last_buoy_force,
            get_last_buoy_point=get_last_buoy_point,
            ros2_sensor_hz=float(ros2_sensor_hz),
            viewer_fps=float(viewer_fps),
            timestep=float(timestep),
            thruster_force_max=float(thruster_force_max),
        ).run(viewer)


__all__ = ["run_headless_runtime", "run_viewer_runtime"]
