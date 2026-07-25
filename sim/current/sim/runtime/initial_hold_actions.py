"""Pose hold and release actions for initial-depth hold state."""

from __future__ import annotations

from sim.runtime.initial_hold_pose import apply_initial_hold_pose_or_depth, capture_initial_hold_pose
from sim.runtime.initial_hold_release import apply_release_velocity, reset_release_state


class InitialHoldActionMixin:
    def capture_pose(
        self,
        *,
        data,
        world_qpos_adr: int,
        water_surface_z: float,
        depth_m: float | None = None,
    ) -> None:
        """Capture the current free-joint pose for repeated hold enforcement."""
        self.pose_qpos = capture_initial_hold_pose(
            data=data,
            world_qpos_adr=world_qpos_adr,
            water_surface_z=water_surface_z,
            depth_m=depth_m,
        )

    def apply(
        self,
        *,
        data,
        mujoco,
        model,
        world_qpos_adr: int,
        world_qvel_adr: int,
        set_bar30_depth,
        set_base_depth,
    ) -> None:
        """Apply the initial depth hold pose/depth constraint if active."""
        apply_initial_hold_pose_or_depth(
            active=self.active,
            depth_m=self.depth_m,
            bar30_depth_m=self.bar30_depth_m,
            pose_qpos=self.pose_qpos,
            data=data,
            mujoco=mujoco,
            model=model,
            world_qpos_adr=world_qpos_adr,
            world_qvel_adr=world_qvel_adr,
            set_bar30_depth=set_bar30_depth,
            set_base_depth=set_base_depth,
        )

    def reset_release_state(self, *, data, mujoco, model, world_qvel_adr: int) -> None:
        """Clear free-joint velocity before release velocity is applied."""
        reset_release_state(data=data, mujoco=mujoco, model=model, world_qvel_adr=world_qvel_adr)

    def apply_release_velocity(
        self,
        *,
        data,
        mujoco,
        model,
        base_id: int,
        world_qvel_adr: int,
    ) -> None:
        """Apply captured body-frame release velocities to MuJoCo qvel."""
        apply_release_velocity(
            data=data,
            mujoco=mujoco,
            model=model,
            base_id=base_id,
            world_qvel_adr=world_qvel_adr,
            velocity_body=self.release_linear_velocity_body,
            angular_velocity_body=self.release_angular_velocity_body,
        )

    def mark_released(self, sim_time: float) -> None:
        """Mark hold inactive and record the release sim time."""
        self.active = False
        self.pending_release_reason = None
        self.release_sim_time = float(sim_time)

    def request_release(self, reason: str) -> bool:
        """Queue a release request for the simulation thread."""
        if not self.active:
            return False
        self.pending_release_reason = str(reason or "requested")
        return True

    def consume_release_request(self) -> str | None:
        """Return and clear one queued release request."""
        reason = self.pending_release_reason
        self.pending_release_reason = None
        return reason


__all__ = ["InitialHoldActionMixin"]
