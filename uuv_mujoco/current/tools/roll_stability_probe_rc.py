"""RC override publishing helpers for the roll stability probe."""

from __future__ import annotations

from roll_stability_rc_frame import build_roll_stability_rc_frame
from roll_stability_rc_spin import spin_rc_publish_loop


class RollStabilityRcMixin:
    def publish_rc(
        self,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        forward: float = 0.0,
        sway: float = 0.0,
        yaw: float = 0.0,
        heave: float = 0.0,
    ) -> None:
        self.rc_pub.publish(
            build_roll_stability_rc_frame(
                roll=roll,
                pitch=pitch,
                forward=forward,
                sway=sway,
                yaw=yaw,
                heave=heave,
            )
        )

    def neutral_rc(self) -> None:
        self.publish_rc()

    def spin_neutral(self, duration: float, hz: float = 20.0) -> None:
        self.spin_rc(duration, hz=hz)

    def spin_rc(
        self,
        duration: float,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        forward: float = 0.0,
        sway: float = 0.0,
        yaw: float = 0.0,
        heave: float = 0.0,
        hz: float = 20.0,
    ) -> None:
        spin_rc_publish_loop(
            self,
            duration,
            publish_once=lambda: self.publish_rc(
                roll=roll,
                pitch=pitch,
                forward=forward,
                sway=sway,
                yaw=yaw,
                heave=heave,
            ),
            hz=hz,
        )


__all__ = ["RollStabilityRcMixin"]
