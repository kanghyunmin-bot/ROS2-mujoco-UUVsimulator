"""MuJoCo ray geometry for the four Water Linked A50 bottom-track beams."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np


BeamRanges = tuple[float | None, float | None, float | None, float | None]
BeamIncidences = tuple[float, float, float, float]
# MuJoCo group 5 is reserved by the active scenes for transparent water and
# other rendering-only geometry.  A DVL ray must see physical terrain and
# obstacles, not the boundary of the visual water volume.
DVL_RAY_GEOM_GROUPS = np.asarray((1, 1, 1, 1, 1, 0), dtype=np.uint8)


def dvl_beam_geometry_from_mujoco(
    bridge: Any,
    data: mujoco.MjData,
) -> tuple[BeamRanges | None, BeamIncidences | None]:
    """Raycast all A50 beams and return range [m] and incidence cosine."""

    sensor_model = getattr(bridge, "_dvl_sensor_model", None)
    site_id = int(getattr(bridge, "_dvl_site_id", -1))
    if sensor_model is None or site_id < 0:
        return None, None

    origin_world = np.asarray(data.site_xpos[site_id], dtype=np.float64).copy()
    rotation_world_dvl = np.asarray(
        data.site_xmat[site_id],
        dtype=np.float64,
    ).reshape(3, 3)
    body_exclude = int(getattr(bridge, "_base_id", -1))
    ranges: list[float | None] = []
    incidences: list[float] = []

    for direction_dvl in sensor_model.directions_frd:
        direction_world = rotation_world_dvl @ np.asarray(
            direction_dvl,
            dtype=np.float64,
        )
        direction_world /= np.linalg.norm(direction_world)
        hit_geom = np.array((-1,), dtype=np.int32)
        hit_normal_world = np.zeros(3, dtype=np.float64)
        distance_m = float(
            mujoco.mj_ray(
                bridge.model,
                data,
                origin_world,
                direction_world,
                DVL_RAY_GEOM_GROUPS,
                1,
                body_exclude,
                hit_geom,
                hit_normal_world,
            )
        )
        if distance_m < 0.0 or not np.isfinite(distance_m):
            ranges.append(None)
            incidences.append(0.0)
            continue
        normal_norm = float(np.linalg.norm(hit_normal_world))
        incidence = (
            0.0
            if normal_norm <= 1.0e-12
            else abs(float(np.dot(direction_world, hit_normal_world / normal_norm)))
        )
        ranges.append(distance_m)
        incidences.append(float(np.clip(incidence, 0.0, 1.0)))

    return tuple(ranges), tuple(incidences)


def dvl_beam_geometry_if_capture_due(
    bridge: Any,
    data: mujoco.MjData,
    sim_t: float,
) -> tuple[BeamRanges | None, BeamIncidences | None]:
    """Avoid raycasts on bridge updates where no A50 capture is scheduled."""

    timing = getattr(bridge, "_dvl_sensor_timing", None)
    if timing is None:
        return None, None
    epsilon_s = timing.config.schedule.epsilon_s
    last_advance_time_s = getattr(
        bridge,
        "_dvl_sensor_last_advance_time_s",
        None,
    )
    rewound = (
        last_advance_time_s is not None
        and sim_t + epsilon_s < float(last_advance_time_s)
    )
    needs_anchor = bool(getattr(bridge, "_dvl_sensor_needs_time_anchor", False))
    if (
        not rewound
        and not needs_anchor
        and sim_t + epsilon_s < timing.next_capture_time_s
    ):
        return None, None
    return dvl_beam_geometry_from_mujoco(bridge, data)


__all__ = [
    "BeamIncidences",
    "BeamRanges",
    "DVL_RAY_GEOM_GROUPS",
    "dvl_beam_geometry_from_mujoco",
    "dvl_beam_geometry_if_capture_due",
]
