"""Initial-depth hold runtime wiring for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .initial_depth_release import release_initial_depth_hold_runtime
from .initial_depth_service import install_initial_depth_hold_service


@dataclass
class InitialDepthHoldRuntime:
    """Bind initial-depth hold state to a MuJoCo model/data pair."""

    state: Any
    data: Any
    mujoco: Any
    model: Any
    base_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    set_bar30_depth: Callable[..., None]
    set_base_depth: Callable[..., None]
    release_actuator_seed: Callable[[], None] | None = None

    def apply_hold(self) -> None:
        """Apply the hold pose/depth constraint when active."""
        self.state.apply(
            data=self.data,
            mujoco=self.mujoco,
            model=self.model,
            world_qpos_adr=self.world_qpos_adr,
            world_qvel_adr=self.world_qvel_adr,
            set_bar30_depth=self.set_bar30_depth,
            set_base_depth=self.set_base_depth,
        )

    def reset_release_state(self) -> None:
        """Clear free-joint velocity before applying release velocity."""
        self.state.reset_release_state(
            data=self.data,
            mujoco=self.mujoco,
            model=self.model,
            world_qvel_adr=self.world_qvel_adr,
        )

    def apply_release_velocity_state(self) -> None:
        """Apply captured body-frame release velocities to MuJoCo state."""
        self.state.apply_release_velocity(
            data=self.data,
            mujoco=self.mujoco,
            model=self.model,
            base_id=self.base_id,
            world_qvel_adr=self.world_qvel_adr,
        )

    def seed_release_actuator_state(self) -> None:
        """Optionally seed actuator internal state at real-start release."""
        if self.release_actuator_seed is None:
            return
        self.release_actuator_seed()

    def release(self, reason: str, *, ros_bridge=None) -> bool:
        """Release the initial-depth hold and publish one fresh sensor snapshot."""
        return release_initial_depth_hold_runtime(self, reason, ros_bridge=ros_bridge)

    def request_release(self, reason: str) -> bool:
        """Queue a release request for the simulation thread."""
        if hasattr(self.state, "request_release"):
            return bool(self.state.request_release(reason))
        if not self.state["active"]:
            return False
        self.state["pending_release_reason"] = str(reason or "requested")
        return True

    def process_pending_release(self, *, ros_bridge=None) -> bool:
        """Apply one queued release request on the simulation thread."""
        if hasattr(self.state, "consume_release_request"):
            reason = self.state.consume_release_request()
        else:
            reason = self.state.get("pending_release_reason")
            self.state["pending_release_reason"] = None
        if reason is None:
            return False
        return self.release(str(reason), ros_bridge=ros_bridge)


__all__ = ["InitialDepthHoldRuntime", "install_initial_depth_hold_service"]
