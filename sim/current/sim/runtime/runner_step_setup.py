"""SimulationStepRuntime wiring for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Callable

from sim.runtime.simulation_step_runtime import SimulationStepRuntime
from sim.runtime.thruster_release_seed import seed_thruster_state_from_current_targets


@dataclass
class RunnerStepSetup:
    step_runtime: SimulationStepRuntime
    run_step: Callable
    publish_ros_once: Callable[[], None]
    publish_qgc_video_once: Callable[[], None]
    viewer_key_callback: Callable


def create_runner_step_setup(
    *,
    args,
    mujoco_module,
    initial_setup,
    control_setup,
    physics_setup,
    plant_replay_direct_rcout: bool,
) -> RunnerStepSetup:
    physics = physics_setup.physics
    model_io = physics_setup.model_io
    if _seed_release_actuators_enabled(plant_replay_direct_rcout):
        initial_setup.initial_depth_runtime.release_actuator_seed = (
            lambda: seed_thruster_state_from_current_targets(
                thruster_actuator_runtime=physics.thruster_actuator_runtime,
                base_id=initial_setup.base_id,
                vertical_names=model_io.vertical_names,
                horizontal_names=model_io.yaw_names,
                horizontal_order=model_io.horizontal_order,
                horizontal_allocator=physics.horizontal_allocator,
            )
        )

    def viewer_key_callback(keycode):
        control_setup.viewer_controls.handle_key(keycode)

    def publish_ros_once() -> None:
        control_setup.ros_bridge_runtime.publish_once(
            data=initial_setup.data,
            initial_depth_hold=initial_setup.initial_depth_hold,
            real_start_status=control_setup.real_start_status,
        )

    def publish_qgc_video_once() -> None:
        control_setup.ros_bridge_runtime.publish_qgc_video_once(
            qgc_video=model_io.qgc_video,
            data=initial_setup.data,
        )

    step_runtime = SimulationStepRuntime(
        mujoco=mujoco_module,
        model=initial_setup.model,
        data=initial_setup.data,
        sitl_enabled=bool(args.sitl),
        plant_replay_direct_rcout=plant_replay_direct_rcout,
        sitl_allow_direct_cmd=control_setup.sitl_allow_direct_cmd,
        command_state=control_setup.command_state,
        sitl_servo_runtime=physics.sitl_servo_runtime,
        thr_target=physics.thr_target,
        sitl_servo_timeout_s=physics.sitl_servo_timeout_s,
        sitl_servo_pwm_values=physics.sitl_servo_pwm_values,
        initial_depth_hold=initial_setup.initial_depth_hold,
        initial_depth_hold_auto_release=initial_setup.initial_depth_hold_auto_release,
        get_ros_bridge=control_setup.ros_bridge_runtime.get,
        release_initial_depth_hold=control_setup.release_initial_depth_hold,
        process_pending_initial_depth_release=control_setup.process_pending_initial_depth_release,
        thruster_update_due=physics.thruster_update_due,
        spin_ros_once=control_setup.ros_bridge_runtime.spin_once,
        apply_direct_command_targets=physics.apply_direct_command_targets,
        update_thruster_forces=physics.update_thruster_forces,
        update_propeller_visuals=physics.update_propeller_visuals,
        apply_initial_depth_hold=initial_setup.apply_initial_depth_hold,
        apply_underwater_wrench=physics.apply_underwater_wrench,
        emit_thruster_debug=physics.emit_thruster_debug,
        enforce_descent_contract=physics.enforce_descent_contract,
        publish_ros_once=publish_ros_once,
        publish_qgc_video_once=publish_qgc_video_once,
    )
    return RunnerStepSetup(
        step_runtime=step_runtime,
        run_step=step_runtime.run_step,
        publish_ros_once=publish_ros_once,
        publish_qgc_video_once=publish_qgc_video_once,
        viewer_key_callback=viewer_key_callback,
    )


def _seed_release_actuators_enabled(plant_replay_direct_rcout: bool) -> bool:
    if not plant_replay_direct_rcout:
        return False
    value = os.environ.get("UUV_REAL_START_SEED_THRUSTER_STATE", "").strip().lower()
    return value in {"1", "true", "yes", "on"}


__all__ = ["RunnerStepSetup", "create_runner_step_setup"]
