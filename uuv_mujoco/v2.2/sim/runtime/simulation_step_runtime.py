"""One-step MuJoCo/SITL execution runtime."""

from __future__ import annotations

from .simulation_step_direct import run_direct_command_runtime_step
from .simulation_step_hold_release import maybe_release_initial_depth_hold
from .simulation_step_raw_pwm import run_raw_pwm_runtime_step
from .simulation_step_state import SimulationStepRuntimeState


class SimulationStepRuntime(SimulationStepRuntimeState):
    """Own one simulation tick without owning setup or shutdown resources."""

    @property
    def raw_pwm_mode(self) -> bool:
        return bool(self.sitl_enabled or self.plant_replay_direct_rcout)

    def run_step(self, is_paused: bool, publish_ros: bool = True) -> tuple[float, float, float, float]:
        """Run one control, physics, diagnostics, and optional publish cycle."""

        self.spin_ros_once()
        thruster_due, thruster_dt = self.thruster_update_due()
        if self.raw_pwm_mode:
            return self.run_raw_pwm_step(is_paused, publish_ros, thruster_due, thruster_dt)

        return run_direct_command_runtime_step(
            self,
            is_paused=is_paused,
            thruster_due=thruster_due,
            thruster_dt=thruster_dt,
            publish_ros=publish_ros,
        )

    def run_raw_pwm_step(
        self,
        is_paused: bool,
        publish_ros: bool,
        thruster_due: bool,
        thruster_dt: float,
    ) -> tuple[float, float, float, float]:
        """Run the raw-PWM path where SITL or replayed RCOU owns the plant."""

        if not self.raw_pwm_mode:
            raise AssertionError("run_raw_pwm_step called outside SITL/plant replay PWM mode")
        return run_raw_pwm_runtime_step(
            self,
            is_paused=is_paused,
            publish_ros=publish_ros,
            thruster_due=thruster_due,
            thruster_dt=thruster_dt,
        )

    def maybe_auto_release_initial_depth_hold(self) -> None:
        maybe_release_initial_depth_hold(
            initial_depth_hold=self.initial_depth_hold,
            auto_release=self.initial_depth_hold_auto_release,
            ros_bridge=self.get_ros_bridge(),
            sitl_servo_pwm_values=self.sitl_servo_pwm_values,
            release_initial_depth_hold=self.release_initial_depth_hold,
        )


__all__ = ["SimulationStepRuntime"]
