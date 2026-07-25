"""Mass, CoM, and inertia console output for physics contract audits."""

from __future__ import annotations

from physics_contract_types import BodyContract


def print_mass_section(body_contract: BodyContract, *, vehicle_mass: float, gravity: float) -> None:
    print(f"  mass={vehicle_mass:.3f} kg weight={vehicle_mass * gravity:.3f} N")
    print(
        "  runtime body contract: "
        f"xml_mass={body_contract.xml_mass_kg:.3f}kg -> "
        f"runtime_mass={body_contract.runtime_mass_kg:.3f}kg"
    )
    print(
        "  runtime CoM: "
        f"xml=({body_contract.xml_com_x_m:+.4f},"
        f"{body_contract.xml_com_y_m:+.4f},"
        f"{body_contract.xml_com_z_m:+.4f})m -> "
        f"runtime=({body_contract.runtime_com_x_m:+.4f},"
        f"{body_contract.runtime_com_y_m:+.4f},"
        f"{body_contract.runtime_com_z_m:+.4f})m"
    )
    print(
        "  runtime inertia diag: "
        f"xml=({body_contract.xml_inertia_x:.6f},"
        f"{body_contract.xml_inertia_y:.6f},"
        f"{body_contract.xml_inertia_z:.6f}) -> "
        f"runtime=({body_contract.runtime_inertia_x:.6f},"
        f"{body_contract.runtime_inertia_y:.6f},"
        f"{body_contract.runtime_inertia_z:.6f}) "
        f"scale=({body_contract.inertia_scale_x:.3f},"
        f"{body_contract.inertia_scale_y:.3f},"
        f"{body_contract.inertia_scale_z:.3f})"
    )


__all__ = ["print_mass_section"]
