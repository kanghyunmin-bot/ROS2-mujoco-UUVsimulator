"""Top-level runtime loop entrypoint for headless and viewer execution."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

from .runtime_loop_modes import run_headless_runtime, run_viewer_runtime
from .runtime_loop_shutdown import ShutdownCallback, run_shutdown_callbacks


def run_simulation_runtime_loop(
    *,
    args: Any,
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
    shutdown_callbacks: Sequence[ShutdownCallback],
) -> None:
    """Run the selected execution loop and always release owned resources."""

    try:
        if args.headless:
            run_headless_runtime(
                stop_event=stop_event,
                viewer_controls=viewer_controls,
                timestep=float(timestep),
                run_step=run_step,
            )
            return

        run_viewer_runtime(
            mujoco_module=mujoco_module,
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
            viewer_key_callback=viewer_key_callback,
        )
    finally:
        run_shutdown_callbacks(shutdown_callbacks)
        stop_event.set()
