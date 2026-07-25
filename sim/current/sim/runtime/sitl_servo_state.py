"""Stateful SITL/plant-replay PWM servo runtime."""

from __future__ import annotations

from .sitl_servo_factory import build_sitl_servo_runtime_kwargs
from .sitl_servo_labels import sitl_servo_mapping_label
from .sitl_servo_packet_apply import apply_servo_packet
from .sitl_servo_state_data import SitlServoRuntimeState
from .sitl_servo_target_apply import apply_servo_commands_to_targets


class SitlServoRuntime(SitlServoRuntimeState):
    @classmethod
    def create(
        cls,
        *,
        all_thruster_names: list[str],
        raw_map: list[str],
        servo_signs: list[float],
        sitl_servo_scale: float,
        timeout_s: float = 0.8,
    ) -> "SitlServoRuntime":
        return cls(**build_sitl_servo_runtime_kwargs(
            all_thruster_names=list(all_thruster_names),
            raw_map=list(raw_map),
            servo_signs=list(servo_signs),
            sitl_servo_scale=sitl_servo_scale,
            timeout_s=timeout_s,
        ))

    def on_packet(self, pwm_values: list[int]) -> None:
        with self.lock:
            apply_servo_packet(self, pwm_values)

    def mapping_label(self) -> str:
        return sitl_servo_mapping_label(self.raw_map, self.servo_signs)

    def apply_to_targets(
        self,
        targets: dict[str, float],
        *,
        now_wall: float,
        timeout_s: float,
    ) -> bool:
        """Copy recent raw PWM commands into thruster targets.

        Returns true when the packet stream is stale and targets were cleared.
        """
        with self.lock:
            return apply_servo_commands_to_targets(
                self,
                targets,
                now_wall=now_wall,
                timeout_s=timeout_s,
            )


__all__ = ["SitlServoRuntime"]
