"""Visual-only propeller actuator updates."""

from __future__ import annotations

import math
from typing import Any


def update_propeller_visuals(runtime: Any, dt: float) -> None:
    # Visual-only propeller spin; vehicle reaction torque is applied in force updates.
    rotor_rpm = getattr(runtime, "rotor_rpm", {})
    for name in runtime.all_thruster_names:
        qadr = runtime.prop_qpos_adr.get(name)
        dadr = runtime.prop_dof_adr.get(name)
        if qadr is None or dadr is None:
            continue
        aid = runtime.actuator_ids[name]
        spin_sign = runtime.prop_spin_sign.get(name, 1.0)
        if name in rotor_rpm:
            omega = spin_sign * rotor_rpm[name] * (math.pi / 30.0)
        else:
            omega = spin_sign * float(runtime.data.ctrl[aid]) * runtime.spin_gain
        if dt > 0.0:
            runtime.prop_phase[name] += omega * dt
        runtime.data.qpos[qadr] = runtime.prop_phase[name]
        runtime.data.qvel[dadr] = 0.0


__all__ = ["update_propeller_visuals"]
