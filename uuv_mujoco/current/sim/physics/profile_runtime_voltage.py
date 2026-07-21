"""Thruster-voltage selection for runtime simulation profiles."""

from __future__ import annotations


def select_profile_thruster_voltage(*, sim_profile: dict, override_voltage: float | None) -> float:
    profile_thruster_voltage = float(sim_profile.get("thruster_voltage", 20.0))
    if override_voltage is None:
        print(
            f"[profile] using thruster_voltage={profile_thruster_voltage:.1f}V from profile",
            flush=True,
        )
        return profile_thruster_voltage

    active_thruster_voltage = float(override_voltage)
    print(
        f"[profile] override thruster_voltage={active_thruster_voltage:.1f}V "
        f"(profile {profile_thruster_voltage:.1f}V)",
        flush=True,
    )
    return active_thruster_voltage


__all__ = ["select_profile_thruster_voltage"]
