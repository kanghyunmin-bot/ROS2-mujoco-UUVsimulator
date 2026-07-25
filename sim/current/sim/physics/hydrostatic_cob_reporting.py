"""CoB runtime override logging and site alignment."""

from __future__ import annotations

from typing import Any, Callable

from .hydrostatic_runtime_types import HydrostaticRuntimeValues


def log_cob_runtime_override(
    *,
    values: HydrostaticRuntimeValues,
    hydro_cfg: Any,
    log: Callable[[str], None],
) -> None:
    if (
        abs(values.cob_longitudinal_offset - values.profile_cob_x_offset) > 1.0e-9
        or abs(values.cob_vertical_offset - values.profile_cob_z_offset) > 1.0e-9
        or abs(values.cob_torque_scale - hydro_cfg.cob_torque_scale) > 1.0e-9
    ):
        log(
            "[physics] CoB runtime override: "
            f"x_offset={values.cob_longitudinal_offset:+.4f}m "
            f"z_offset={values.cob_vertical_offset:+.4f}m "
            f"torque_scale={values.cob_torque_scale:.4f}"
        )


def align_cob_site(
    *,
    model: Any,
    base_id: int,
    cob_site_id: int,
    values: HydrostaticRuntimeValues,
    log: Callable[[str], None],
) -> None:
    if cob_site_id < 0:
        return

    target_x = float(model.body_ipos[base_id][0] + values.cob_longitudinal_offset)
    target_z = float(model.body_ipos[base_id][2] + values.cob_vertical_offset)
    old_x = float(model.site_pos[cob_site_id][0])
    old_z = float(model.site_pos[cob_site_id][2])
    if abs(target_x - old_x) < 1e-6 and abs(target_z - old_z) < 1e-6:
        return

    model.site_pos[cob_site_id][0] = target_x
    model.site_pos[cob_site_id][2] = target_z
    log(
        f"[model] aligned CoB: x {old_x:.4f} -> {target_x:.4f} "
        f"(offset={values.cob_longitudinal_offset:+.4f}), "
        f"z {old_z:.4f} -> {target_z:.4f} "
        f"(offset={values.cob_vertical_offset:+.4f})"
    )


__all__ = ["align_cob_site", "log_cob_runtime_override"]
