"""Hydrostatic and start-depth console output for physics contract audits."""

from __future__ import annotations

from typing import Any


def print_hydrostatic_section(
    *,
    rho: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
) -> None:
    print(f"  rho={rho:.1f} kg/m^3 neutral_volume={neutral_volume:.6f} m^3")
    print(f"  buoyancy_scale={buoyancy_scale:.6f}")
    print(
        "  hydrostatic: "
        f"source={hydro_cfg.hydrostatic_volume_source} "
        f"restoring_active={bool(hydro_cfg.hydrostatic_restoring_active)} "
        f"roll_k={hydro_cfg.hydrostatic_restoring_roll_stiffness:.3f} "
        f"pitch_k={hydro_cfg.hydrostatic_restoring_pitch_stiffness:.3f} N*m/rad"
    )


def print_start_depth_section(
    *,
    auto_bar30_depth: float,
    auto_base_depth: float,
    start_candidates: list[tuple[str, float]],
) -> None:
    print(f"  auto_start: bar30_depth={auto_bar30_depth:.3f} m base_depth={auto_base_depth:.3f} m")
    print("  top start-depth candidates:")
    for name, depth in sorted(start_candidates, key=lambda item: item[1], reverse=True)[:10]:
        print(f"    {name:36s} {depth:.3f} m Bar30")


__all__ = ["print_hydrostatic_section", "print_start_depth_section"]
