"""Public Bar30 depth methods for MuJoCoBaseState."""

from __future__ import annotations

from sim.runtime.base_state_bar30 import (
    bar30_depth_now_m as _bar30_depth_now_m,
    bar30_world_z as _bar30_world_z,
    set_bar30_depth as _set_bar30_depth,
)


class BaseStateBar30Mixin:
    def bar30_world_z(self) -> float:
        """Return the Bar30 pressure sensor world z, falling back to base origin."""
        return _bar30_world_z(data=self.data, base_id=self.base_id, bar30_site_id=self.bar30_site_id)

    def bar30_depth_now_m(self) -> float:
        """Return the current positive-down Bar30 depth."""
        return _bar30_depth_now_m(
            data=self.data,
            water_surface_z=self.water_surface_z,
            base_id=self.base_id,
            bar30_site_id=self.bar30_site_id,
        )

    def set_bar30_depth(self, depth_m: float, reset_velocity: bool = True) -> None:
        """Translate the free body so bar30_site is at a positive-down depth."""
        _set_bar30_depth(
            mujoco=self.mujoco,
            model=self.model,
            data=self.data,
            world_qpos_adr=self.world_qpos_adr,
            world_qvel_adr=self.world_qvel_adr,
            water_surface_z=self.water_surface_z,
            base_id=self.base_id,
            bar30_site_id=self.bar30_site_id,
            depth_m=depth_m,
            reset_velocity=reset_velocity,
        )


__all__ = ["BaseStateBar30Mixin"]
