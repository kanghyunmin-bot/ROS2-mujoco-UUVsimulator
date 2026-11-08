"""Audit recorded sensor frames and FCU parameter changes without ROS transport.

The optional rosbags package is shared with extract_rosbag_trends. Recorded TF
expresses the declared mount, so an unrecorded hardware movement remains unknown.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from collections import Counter
from pathlib import Path

import numpy as np


def transform_matrix(xyz: list[float], quat: list[float]) -> np.ndarray:
    """Return the parent-from-child rigid transform, with translation [m]."""
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError("Invalid TF quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    position = np.asarray(xyz, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("Invalid TF translation")
    matrix[:3, 3] = position
    return matrix


def resolve_transform(
    edges: dict[str, list[dict]],
    child: str,
    base: str = "base_link",
    *,
    dynamic_children: set[str] | None = None,
) -> dict:
    """Compose the recorded chain into base coordinates [m], retaining ambiguity."""
    child, base = child.strip("/"), base.strip("/")
    matrix = np.eye(4)
    chain = []
    while child != base:
        if child in chain:
            return {"status": "cycle", "chain": chain}
        chain.append(child)
        if dynamic_children and child in dynamic_children:
            return {"status": "time_dependent_mount", "chain": chain}
        candidates = edges.get(child, [])
        if not candidates:
            return {"status": "missing", "chain": chain}
        edge = candidates[0]
        observed = transform_matrix(edge["xyz_m"], edge["quat_xyzw"])
        for candidate in candidates[1:]:
            other = transform_matrix(candidate["xyz_m"], candidate["quat_xyzw"])
            if candidate["parent"] != edge["parent"] or not np.allclose(
                other, observed, rtol=0.0, atol=1e-9
            ):
                return {"status": "conflicting_recorded_mount", "chain": chain}
        matrix = observed @ matrix
        child = edge["parent"]
    return {"status": "recorded_static_tf", "chain": chain, "base_from_sensor": matrix.tolist()}


def sensor_point_velocity(
    velocity: np.ndarray, omega: np.ndarray, lever: np.ndarray
) -> np.ndarray:
    """Shift body-frame velocity [m/s] by omega [rad/s] cross lever [m]."""
    return np.asarray(velocity) + np.cross(omega, lever)


def sensor_point_acceleration(
    acceleration: np.ndarray,
    omega: np.ndarray,
    angular_acceleration: np.ndarray,
    lever: np.ndarray,
) -> np.ndarray:
    """Shift acceleration [m/s²] using omega [rad/s], alpha [rad/s²], lever [m]."""
    return (
        np.asarray(acceleration)
        + np.cross(angular_acceleration, lever)
        + np.cross(omega, np.cross(omega, lever))
    )


def audit(database: Path) -> dict:
    """Read a ROS 2 SQLite bag and preserve all static-TF publications."""
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(
        get_types_from_msg(
            "std_msgs/Header header\nstring param_id\nrcl_interfaces/ParameterValue value\n"
            "uint16 param_index\nuint16 param_count",
            "mavros_msgs/msg/ParamEvent",
        )
    )
    root = Path(__file__).resolve().parents[3]
    definitions = {}
    for package in ("dvl_msgs", "auv_dvl_a50_msg"):
        for path in (root / "rospkg/src" / package / "msg").glob("*.msg"):
            definitions.update(get_types_from_msg(path.read_text(), f"{package}/msg/{path.stem}"))
    store.register(definitions)
    edges = {}
    frames = {}
    dynamic = {}
    parameters = {}
    timeline = []
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as connection:
        if connection.execute("PRAGMA quick_check").fetchone()[0] != "ok":
            raise ValueError("SQLite integrity check failed")
        origin = connection.execute("SELECT min(timestamp) FROM messages").fetchone()[0]
        if origin is None:
            raise ValueError("Empty bag")
        wanted = {
            "/tf_static", "/tf", "/mavros/imu/data", "/dvl/data", "/depth/pose",
            "/mavros/param/event",
        }
        for tid, topic, typ in connection.execute("SELECT id,name,type FROM topics").fetchall():
            if topic not in wanted:
                continue
            observed = Counter()
            for timestamp, blob in connection.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (tid,)
            ):
                message = store.deserialize_cdr(blob, typ)
                if topic in {"/tf_static", "/tf"}:
                    for item in message.transforms:
                        parent = item.header.frame_id.strip("/")
                        child = item.child_frame_id.strip("/")
                        if topic == "/tf":
                            key = parent + "->" + child
                            dynamic[key] = dynamic.get(key, 0) + 1
                            continue
                        v, q = item.transform.translation, item.transform.rotation
                        edge = {
                            "parent": parent, "child": child,
                            "xyz_m": [v.x, v.y, v.z],
                            "quat_xyzw": [q.x, q.y, q.z, q.w],
                        }
                        if edge not in edges.setdefault(child, []):
                            edges[child].append(edge)
                elif topic == "/mavros/param/event":
                    value = message.value
                    if value.type not in (1, 2, 3):
                        continue
                    scalar_values = {
                        1: value.bool_value, 2: value.integer_value,
                        3: value.double_value,
                    }
                    result = scalar_values[value.type]
                    parameters[message.param_id] = result
                    timeline.append({
                        "time_s": (timestamp - origin) / 1e9,
                        "name": message.param_id, "value": result,
                        "index": int(message.param_index),
                        "reported_count": int(message.param_count),
                    })
                else:
                    observed[message.header.frame_id] += 1
            if observed:
                frames[topic] = dict(observed)
    return {
        "schema": "uuv.real2sim.bag_contract.v1", "source": str(database.resolve()),
        "record_origin_ns": origin, "message_frames": frames, "static_edges": edges,
        "dynamic_edge_counts": dynamic,
        "sensor_transforms": {
            child: resolve_transform(
                edges, child,
                dynamic_children={key.split("->", 1)[1] for key in dynamic},
            )
            for child in sorted(
                {"fcu_link", "imu_link", "dvl", "depth_link"}
                | {frame.strip("/") for observed in frames.values() for frame in observed if frame.strip("/")}
            )
        },
        "fcu_parameters_last_observed": parameters, "fcu_parameter_timeline": timeline,
        "scope": (
            "Recorded mounts and parameter observations. Parameters from another "
            "bag need explicit provenance. Dynamic sensor mounts require "
            "time-dependent resolution. Physical moves without TF updates "
            "remain unknown."
        ),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = audit(args.database)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({
        "source": result["source"],
        "parameter_count": len(result["fcu_parameters_last_observed"]),
        "sensor_transforms": result["sensor_transforms"],
    }, indent=2))
