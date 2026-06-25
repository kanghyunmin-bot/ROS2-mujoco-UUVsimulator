"""Final runtime-loop invocation for the MuJoCo runner."""

from __future__ import annotations

from sim.runtime.runtime_loop_entry import run_simulation_runtime_loop


def run_configured_simulation_loop(
    *,
    args,
    mujoco_module,
    initial_setup,
    control_setup,
    physics_setup,
    step_setup,
) -> None:
    model_io = physics_setup.model_io
    physics = physics_setup.physics
    run_simulation_runtime_loop(
        args=args,
        mujoco_module=mujoco_module,
        model=initial_setup.model,
        data=initial_setup.data,
        base_id=initial_setup.base_id,
        act=model_io.actuator_ids,
        camera_ids=model_io.camera_ids,
        thruster_site_ids=physics.thruster_site_ids,
        thruster_names=model_io.vertical_names + model_io.yaw_names,
        sensor_site_ids=model_io.sensor_site_ids,
        viewer_controls=control_setup.viewer_controls,
        stop_event=control_setup.stop_event,
        run_step=step_setup.run_step,
        publish_ros_once=step_setup.publish_ros_once,
        sensor_value=model_io.sensor_value,
        get_last_buoy_force=lambda: physics.underwater_wrench_runtime.last_buoy_force,
        get_last_buoy_point=lambda: physics.underwater_wrench_runtime.last_buoy_point,
        ros2_sensor_hz=float(args.ros2_sensor_hz),
        viewer_fps=float(args.viewer_fps),
        timestep=float(initial_setup.model.opt.timestep),
        thruster_force_max=float(physics.thruster_force_max),
        viewer_key_callback=step_setup.viewer_key_callback,
        shutdown_callbacks=(
            control_setup.ros_bridge_runtime.shutdown,
            model_io.qgc_video.close,
            physics.thruster_debug_runtime.close,
        ),
    )


__all__ = ["run_configured_simulation_loop"]
