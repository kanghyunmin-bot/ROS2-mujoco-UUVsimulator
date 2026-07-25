"""Horizontal thruster allocation for vectored ArduSub geometry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from sim.physics.horizontal_allocator_geometry import (
    horizontal_allocation_matrix,
    row_scaled_pseudo_inverse,
    saturate_allocator_commands,
)


@dataclass(frozen=True)
class HorizontalAllocator:
    order: tuple[str, ...]
    allocation: np.ndarray
    pseudo_inverse: np.ndarray

    @classmethod
    def from_model(
        cls,
        *,
        model: Any,
        mujoco_module: Any,
        actuator_ids: Mapping[str, int],
        base_id: int,
        order: Sequence[str],
    ) -> "HorizontalAllocator":
        order_tuple = tuple(order)
        allocation = horizontal_allocation_matrix(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            base_id=base_id,
            order=order_tuple,
        )
        pseudo_inverse = row_scaled_pseudo_inverse(allocation)
        return cls(order=order_tuple, allocation=allocation, pseudo_inverse=pseudo_inverse)

    def mix(self, forward_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
        wrench_cmd = np.array([forward_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
        commands = self.pseudo_inverse @ wrench_cmd
        return saturate_allocator_commands(commands)

    def yaw_tau_scale(self, thruster_force_max: float) -> float:
        if not self.allocation.size or not self.order:
            return 1.0
        yaw_basis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        unit_commands = self.pseudo_inverse @ yaw_basis
        tau_per_unit = float(np.dot(self.allocation[2], unit_commands) * float(thruster_force_max))
        if abs(tau_per_unit) < 1e-6:
            return 1.0
        return abs(tau_per_unit)
