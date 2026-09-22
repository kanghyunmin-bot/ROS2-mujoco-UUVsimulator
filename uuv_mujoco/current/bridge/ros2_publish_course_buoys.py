"""Live course-buoy status publisher for mission fake-vision control."""

from __future__ import annotations

import json
import math
import os
import re
from typing import Any

import mujoco
import numpy as np


BUOY_FLOAT_RE = re.compile(
    r"^course_buoy_(?:(?P<course>a|b|test_tank)_)?(?P<kind>pinger_)?(?P<color>[a-z]+)_(?P<number>\d+)_float$"
)


def build_course_buoy_status_msg(bridge: Any, data: Any, _stamp: Any, state: Any) -> Any:
    msg = bridge.String()
    msg.data = json.dumps(_course_buoy_status_payload(bridge, bridge.model, data, state), sort_keys=True)
    return msg


def build_course_buoy_publish_builders(bridge: Any, data: Any, stamp: Any, state: Any) -> dict[str, object]:
    return {
        "course_buoy_status": lambda: build_course_buoy_status_msg(bridge, data, stamp, state),
        "collector_state": lambda: build_collector_state_msg(bridge, data, stamp, state),
    }


def build_collector_state_msg(bridge: Any, _data: Any, stamp: Any, state: Any) -> Any:
    """Publish collector events without exposing MuJoCo positions to controllers."""
    msg = bridge.CollectorState()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    runtime = getattr(bridge, "_course_buoy_runtime", None)
    status_fn = getattr(runtime, "status_by_name", None)
    rows = _canonical_runtime_rows(status_fn() if status_fn is not None else {})

    seen = getattr(bridge, "_collector_event_seen", None)
    if seen is None:
        seen = {"detached": set(), "netted": set(), "released": set()}
        bridge._collector_event_seen = seen
    candidates: list[tuple[int, float, str, str]] = []
    for target_id, row in rows.items():
        if bool(row.get("net_score_released", False)) and target_id not in seen["released"]:
            candidates.append((3, float(row.get("net_score_release_time_s", state.sim_t)), "RELEASED", target_id))
        if bool(row.get("netted", False)) and target_id not in seen["netted"]:
            candidates.append((2, float(row.get("netted_time_s", state.sim_t)), "NETTED", target_id))
        release_time = float(row.get("release_time_s", -1.0))
        if release_time >= 0.0 and target_id not in seen["detached"]:
            candidates.append((1, release_time, "DETACHED", target_id))

    if candidates:
        _priority, _event_time, event, target_id = max(candidates, key=lambda item: (item[1], item[0]))
        seen[event.lower()].add(target_id)
        bridge._collector_last_event = (event, target_id)
        # Reliable ROS delivery and an ID-based FSM latch make a short event
        # sufficient, while allowing a batch of score releases to drain quickly.
        bridge._collector_event_until_s = float(state.sim_t) + 0.5
    elif float(state.sim_t) <= float(getattr(bridge, "_collector_event_until_s", -1.0)):
        event, target_id = getattr(bridge, "_collector_last_event", ("IDLE", ""))
    else:
        event, target_id = "IDLE", ""

    msg.target_id = target_id
    msg.captured = event == "NETTED"
    msg.detached = event == "DETACHED"
    msg.netted = event == "NETTED"
    msg.released = event == "RELEASED"
    msg.collector_eq_active = any(
        bool(row.get("collector_eq_active", False)) for row in rows.values()
    )
    capture_states = {str(row.get("capture_state", "FREE")) for row in rows.values()}
    if "NETTING" in capture_states:
        msg.capture_state = "NETTING"
    elif msg.collector_eq_active or "NETTED" in capture_states:
        msg.capture_state = "NETTED"
    elif "SCORE_RELEASED" in capture_states:
        msg.capture_state = "SCORE_RELEASED"
    else:
        msg.capture_state = "FREE"
    msg.state = event
    return msg


def _canonical_runtime_rows(rows: dict[str, dict[str, object]]) -> dict[str, dict[str, object]]:
    """Collapse runtime prefix/body aliases to one physical buoy event ID."""
    canonical: dict[str, dict[str, object]] = {}
    for target_id, row in rows.items():
        float_id = target_id if target_id.endswith("_float") else f"{target_id}_float"
        canonical_id = float_id if float_id in rows else target_id
        canonical[canonical_id] = row
    return canonical


def _course_buoy_status_payload(bridge: Any, model: Any, data: Any, state: Any) -> dict[str, object]:
    return {
        "frame_id": "world",
        "source": "mujoco_live",
        "time_s": float(state.sim_t),
        "buoys": [_course_buoy_row(bridge, model, data, body_id) for body_id in _course_buoy_body_ids(model)],
    }


def _course_buoy_body_ids(model: Any) -> list[int]:
    obj_body = mujoco.mjtObj.mjOBJ_BODY
    body_ids: list[int] = []
    for body_id in range(int(model.nbody)):
        name = mujoco.mj_id2name(model, obj_body, body_id) or ""
        if BUOY_FLOAT_RE.match(name):
            body_ids.append(int(body_id))
    body_ids.sort(key=lambda body_id: mujoco.mj_id2name(model, obj_body, body_id) or "")
    return body_ids


def _course_buoy_row(bridge: Any, model: Any, data: Any, body_id: int) -> dict[str, object]:
    obj_body = mujoco.mjtObj.mjOBJ_BODY
    obj_geom = mujoco.mjtObj.mjOBJ_GEOM
    obj_site = mujoco.mjtObj.mjOBJ_SITE
    obj_equality = mujoco.mjtObj.mjOBJ_EQUALITY
    body_name = mujoco.mj_id2name(model, obj_body, body_id) or f"body_{body_id}"
    prefix = body_name.removesuffix("_float")
    parsed = _parse_buoy_float_name(body_name)
    float_geom_id = mujoco.mj_name2id(model, obj_geom, f"{prefix}_float_geom")
    attach_site_id = mujoco.mj_name2id(model, obj_site, f"{prefix}_attach_site")
    magnet_site_id = mujoco.mj_name2id(model, obj_site, f"{prefix}_magnet_site")
    eq_id = mujoco.mj_name2id(model, obj_equality, f"{prefix}_magnet_weld")
    body_xyz = _vec3(data.xpos[body_id])
    attach_xyz = _site_vec3(data, attach_site_id)
    magnet_xyz = _site_vec3(data, magnet_site_id)
    eq_active = _eq_active(data, eq_id)
    probe_surface_distance_m = _probe_surface_distance_m(model, data, body_xyz)
    probe_release_margin_m = _probe_release_margin_m(model, float_geom_id, probe_surface_distance_m)
    probe_clearance_m = _env_float("UUV_COURSE_BUOY_PROXIMITY_RELEASE_CLEARANCE_M", 0.055)
    has_magnet = attach_site_id >= 0 and magnet_site_id >= 0
    detached = bool((not has_magnet) or (has_magnet and eq_active is False))
    row = {
        "id": body_name,
        "prefix": prefix,
        "course": parsed[0],
        "class_name": parsed[1],
        "color": parsed[1],
        "number": parsed[2],
        "body_xyz": body_xyz,
        "attach_xyz": attach_xyz,
        "magnet_xyz": magnet_xyz,
        # For the collector mission the useful target is the live float
        # center. Attach/magnet sites are still published for diagnostics.
        "target_xyz": body_xyz,
        "target_kind": "float_center",
        "has_magnet": bool(has_magnet),
        "detached": detached,
        "eq_active": eq_active,
        "eq_id": int(eq_id),
        "probe_surface_distance_m": probe_surface_distance_m,
        "probe_release_margin_m": probe_release_margin_m,
        "probe_release_proximity": (
            probe_release_margin_m is not None and probe_release_margin_m <= probe_clearance_m
        ),
    }
    # Privileged reward telemetry only; never part of the VLA observation.
    runtime = getattr(bridge, '_course_buoy_runtime', None)
    if runtime is not None and os.environ.get('UUV_RL_REWARD_TELEMETRY') == '1':
        row['rl_right_fork_stem_contact_active'] = bool(body_id in runtime._right_fork_stem_contacts)
        root_id = runtime.vehicle_root_body_id
        fork_local = np.array([0.3355119380367045, -0.09192734377108369, -0.10434422587321063])
        fork = data.xpos[root_id] + data.xmat[root_id].reshape(3, 3) @ fork_local
        stem_id = mujoco.mj_name2id(model, obj_geom, prefix + '_pvc_pipe')
        if stem_id >= 0:
            stem = data.geom_xpos[stem_id]
            row['rl_height_error_m'] = float(fork[2] - stem[2])
            row['rl_fork_stem_distance_m'] = float(np.linalg.norm(fork - stem))
            # Closest point on the rod centerline, expressed in the robot-fixed
            # slot frame. Unlike center distance, this respects rod length.
            rotation = data.xmat[root_id].reshape(3, 3)
            axis = data.geom_xmat[stem_id].reshape(3, 3)[:, 2]
            half_length = float(model.geom_size[stem_id, 1])
            along = float(np.clip(np.dot(fork-stem, axis), -half_length, half_length))
            closest = stem + along*axis
            row['rl_slot_offset_m'] = (rotation.T @ (closest-fork)).tolist()
        # Several body-local samples allow a partial buoy/rod view. No image
        # area threshold; ray occlusion still excludes points behind obstacles.
        points = [data.xpos[body_id].copy()]
        target_geoms = np.flatnonzero(model.geom_bodyid == body_id)
        for gid in target_geoms:
            name = mujoco.mj_id2name(model, obj_geom, int(gid)) or ''
            if 'float' not in name and not name.endswith('_pvc_pipe'):
                continue
            center = data.geom_xpos[gid]
            axes = data.geom_xmat[gid].reshape(3, 3)
            points.append(center.copy())
            if name.endswith('_pvc_pipe'):
                for amount in (-.9, -.45, .45, .9):
                    points.append(center + axes[:,2]*float(model.geom_size[gid,1])*amount)
            else:
                for j in range(3):
                    for sign in (-1,1):
                        points.append(center+sign*.8*float(model.geom_size[gid,j])*axes[:,j])
        aspect = float(getattr(bridge, "_stereo_image_width",640))/float(getattr(bridge,"_stereo_image_height",360))
        visible = False
        hand_visible = False
        for camera_name in ('stereo_left', 'stereo_right'):
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if camera_id < 0:
                continue
            camera_visible = camera_points_visible(model, data, camera_id, body_id, points, aspect)
            visible |= camera_visible
            if camera_name == 'stereo_right':
                hand_visible = camera_visible
        row['rl_hand_target_line_of_sight'] = hand_visible
        row['rl_target_line_of_sight'] = visible
    row.update(_runtime_buoy_row(bridge, prefix, body_name))
    return row


def camera_points_visible(model, data, camera_id, body_id, points, aspect):
    """Match camera render groups while testing sampled visible target points."""
    render_groups = np.array([1,1,1,0,0,0], dtype=np.uint8)
    window_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'front_optical_window')
    # This named mesh is the camera's transmissive cover, not a task obstacle.
    # Do not exclude the entire vehicle body: its fork and housing occlude.
    window_alpha = float(model.geom_rgba[window_id,3]) if window_id >= 0 else 1.
    if window_id >= 0 and model.geom_matid[window_id] >= 0:
        window_alpha = float(model.mat_rgba[model.geom_matid[window_id],3])
    slope = math.tan(math.radians(float(model.cam_fovy[camera_id])) / 2)
    for point in points:
        delta = point-data.cam_xpos[camera_id]
        local = data.cam_xmat[camera_id].reshape(3,3).T @ delta
        if local[2] >= 0 or abs(local[1]) > -local[2]*slope or abs(local[0]) > -local[2]*slope*aspect:
            continue
        length = float(np.linalg.norm(delta))
        if length <= 0:
            continue
        hit = np.array([-1], dtype=np.int32)
        direction = delta/length
        origin = data.cam_xpos[camera_id].copy()
        travelled = 0.
        for _ in range(8):
            distance = mujoco.mj_ray(model,data,origin,direction,render_groups,1,-1,hit)
            if hit[0] < 0 or travelled + distance > length + 1e-5:
                break
            if int(model.geom_bodyid[hit[0]]) == body_id:
                return True
            if hit[0] != window_id or window_alpha >= .1:
                break
            # Advance through the thin cover's entry and exit surfaces only.
            travelled += distance + 1e-6
            origin += (distance + 1e-6)*direction
    return False


def _runtime_buoy_row(bridge: Any, prefix: str, body_name: str) -> dict[str, object]:
    runtime = getattr(bridge, "_course_buoy_runtime", None)
    status_fn = getattr(runtime, "status_by_name", None)
    if status_fn is None:
        return {
            "collector_net_enabled": False,
            "capture_state": "FREE",
            "netting": False,
            "netted": False,
            "collector_eq_active": False,
            "net_reverse_released": False,
            "net_reverse_release_time_s": -1.0,
            "netted_time_s": -1.0,
            "net_score_released": False,
            "net_score_release_time_s": -1.0,
            "release_time_s": -1.0,
        }
    try:
        rows = status_fn()
        row = rows.get(body_name) or rows.get(prefix) or {}
    except Exception:
        row = {}
    return {
        "collector_net_enabled": bool(row.get("collector_net_enabled", False)),
        "capture_state": str(row.get("capture_state", "FREE")),
        "netting": bool(row.get("netting", False)),
        "netted": bool(row.get("netted", False)),
        "collector_eq_active": bool(row.get("collector_eq_active", False)),
        "net_reverse_released": bool(row.get("net_reverse_released", False)),
        "net_reverse_release_time_s": float(row.get("net_reverse_release_time_s", -1.0)),
        "netted_time_s": float(row.get("netted_time_s", -1.0)),
        "net_score_released": bool(row.get("net_score_released", False)),
        "net_score_release_time_s": float(row.get("net_score_release_time_s", -1.0)),
        "release_time_s": float(row.get("release_time_s", -1.0)),
        "release_reason": str(row.get("release_reason", "")),
        "release_force_n": float(row.get("release_force_n", 0.0)),
        "release_rake_contact": bool(row.get("release_rake_contact", False)),
        "release_hand_contact": bool(row.get("release_hand_contact", False)),
        "release_right_fork_stem_contact": bool(row.get("release_right_fork_stem_contact", False)),
        "release_right_fork_region": bool(row.get("release_right_fork_region", False)),
        "hand_contact_active": bool(row.get("hand_contact_active", False)),
    }


def _parse_buoy_float_name(name: str) -> tuple[str, str, int]:
    match = BUOY_FLOAT_RE.match(name)
    if not match:
        return "", "", 0
    course = match.group("course")
    if match.group("kind"):
        course = "pinger"
    return course or "", match.group("color") or "", int(match.group("number") or 0)


def _site_vec3(data: Any, site_id: int) -> list[float] | None:
    if int(site_id) < 0:
        return None
    return _vec3(data.site_xpos[int(site_id)])


def _eq_active(data: Any, eq_id: int) -> bool | None:
    if int(eq_id) < 0 or not hasattr(data, "eq_active"):
        return None
    return bool(int(data.eq_active[int(eq_id)]))


def _probe_surface_distance_m(model: Any, data: Any, body_xyz: list[float]) -> float | None:
    geom_ids = [
        int(geom_id)
        for geom_id in (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "mission_port_rake_root_probe"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "mission_starboard_rake_root_probe"),
        )
        if int(geom_id) >= 0
    ]
    if not geom_ids:
        return None
    point = np.asarray(body_xyz, dtype=np.float64)
    if point.size < 3 or not np.all(np.isfinite(point[:3])):
        return None
    return min(_point_probe_surface_distance_m(model, data, geom_id, point[:3]) for geom_id in geom_ids)


def _point_probe_surface_distance_m(model: Any, data: Any, geom_id: int, point: np.ndarray) -> float:
    center = np.asarray(data.geom_xpos[int(geom_id)], dtype=np.float64)
    sizes = np.asarray(model.geom_size[int(geom_id)], dtype=np.float64).reshape(-1)
    radius = float(sizes[0]) if sizes.size >= 1 else 0.0
    half_length = float(sizes[1]) if sizes.size >= 2 else 0.0
    if half_length <= 0.0:
        return float(np.linalg.norm(point - center)) - radius
    xmat = np.asarray(data.geom_xmat[int(geom_id)], dtype=np.float64).reshape(3, 3)
    axis = xmat[:, 2]
    norm_axis = float(np.linalg.norm(axis))
    if norm_axis <= 1.0e-9:
        return float(np.linalg.norm(point - center)) - radius
    axis = axis / norm_axis
    a = center - axis * half_length
    b = center + axis * half_length
    segment = b - a
    denom = float(np.dot(segment, segment))
    if denom <= 1.0e-12:
        closest = center
    else:
        t = float(np.clip(np.dot(point - a, segment) / denom, 0.0, 1.0))
        closest = a + t * segment
    return float(np.linalg.norm(point - closest)) - radius


def _probe_release_margin_m(model: Any, float_geom_id: int, surface_distance_m: float | None) -> float | None:
    if surface_distance_m is None or int(float_geom_id) < 0:
        return None
    sizes = np.asarray(model.geom_size[int(float_geom_id)], dtype=np.float64).reshape(-1)
    buoy_radius = float(np.max(sizes[: min(3, sizes.size)])) if sizes.size else 0.055
    return float(surface_distance_m) - buoy_radius


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _vec3(values: Any) -> list[float]:
    vector = np.asarray(values, dtype=np.float64).reshape(-1)
    result = [float(vector[index]) for index in range(min(3, vector.size))]
    while len(result) < 3:
        result.append(math.nan)
    return result


__all__ = [
    "build_collector_state_msg",
    "build_course_buoy_publish_builders",
    "build_course_buoy_status_msg",
]
