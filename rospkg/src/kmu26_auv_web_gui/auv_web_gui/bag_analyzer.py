import argparse
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any


ODOM_TOPICS = {
    "/odometry/filtered": "filtered",
    "/dvl/odometry": "dvl_odom",
}
DVL_DR_TOPIC = "/dvl/position"
DVL_DATA_TOPIC = "/dvl/data"
DVL_TWIST_TOPIC = "/dvl/twist"
TARGET_TOPICS = set(ODOM_TOPICS) | {DVL_DR_TOPIC, DVL_DATA_TOPIC, DVL_TWIST_TOPIC}
ARTIFACT_DIR_NAME = "bag_analysis_results"


def analyze_bag(bag_path: str | Path, max_samples: int = 600) -> dict:
    bag_dir, storage_uri = _resolve_bag_paths(bag_path)

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(f"ROS bag Python modules are not available: {exc}") from exc

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(storage_uri), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(msg_type)
        for topic, msg_type in topic_types.items()
        if topic in TARGET_TOPICS
    }

    topic_counts = {topic: 0 for topic in topic_types}
    tracks = {
        "filtered": [],
        "dvl_odom": [],
        "dvl_dr": [],
    }
    dvl_data = _new_dvl_data_stats()
    dvl_twist = _new_dvl_twist_stats()

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        msg_type = message_types.get(topic)
        if msg_type is None:
            continue

        try:
            msg = deserialize_message(data, msg_type)
        except Exception:
            continue

        t = _message_time(msg, timestamp_ns)
        if topic in ODOM_TOPICS:
            point = _odom_point(msg, t)
            if point:
                tracks[ODOM_TOPICS[topic]].append(point)
        elif topic == DVL_DR_TOPIC:
            point = _dvl_dr_point(msg, t)
            if point:
                tracks["dvl_dr"].append(point)
        elif topic == DVL_DATA_TOPIC:
            _update_dvl_data_stats(dvl_data, msg, t)
        elif topic == DVL_TWIST_TOPIC:
            _update_dvl_twist_stats(dvl_twist, msg, t)

    odometry = {
        name: _track_metrics(points)
        for name, points in tracks.items()
    }
    dvl = {
        "data": _finalize_dvl_data_stats(dvl_data),
        "twist": _finalize_dvl_twist_stats(dvl_twist),
    }
    result = {
        "bag_path": str(bag_dir),
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "topics": {
            topic: {
                "type": topic_types[topic],
                "messages": topic_counts.get(topic, 0),
            }
            for topic in sorted(topic_types)
        },
        "odometry": odometry,
        "dvl": dvl,
        "samples": {
            name: _decimate(points, max_samples)
            for name, points in tracks.items()
        },
    }
    result["assessment"] = _assessment(result)
    return result


def write_analysis_report(result: dict, output_path: str | Path | None = None) -> str:
    bag_dir = Path(result["bag_path"])
    path = Path(output_path) if output_path else bag_dir / "analysis.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return str(path)


def write_analysis_artifacts(
    result: dict,
    output_root: str | Path | None = None,
) -> dict:
    output_dir = _new_artifact_dir(result, output_root)
    result_path = output_dir / "result.json"
    image_path = output_dir / "trajectory.svg"

    result["report_dir"] = str(output_dir)
    result["result_path"] = str(result_path)
    result["image_path"] = str(image_path)

    result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    image_path.write_text(render_trajectory_svg(result), encoding="utf-8")
    return {
        "report_dir": str(output_dir),
        "result_path": str(result_path),
        "image_path": str(image_path),
    }


def render_trajectory_svg(result: dict, width: int = 1000, height: int = 700) -> str:
    tracks = [
        ("filtered", "EKF", "#6ec6ff"),
        ("dvl_odom", "DVL Odom", "#52d273"),
        ("dvl_dr", "DVL DR", "#f0b84f"),
    ]
    samples = result.get("samples", {})
    track_points = [
        (name, label, color, samples.get(name, []))
        for name, label, color in tracks
        if len(samples.get(name, [])) > 1
    ]

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#0b0f0e"/>',
    ]
    if not track_points:
        lines.extend(
            [
                '<text x="40" y="60" fill="#dce8e1" font-size="24" font-family="sans-serif">',
                "No trajectory samples",
                "</text>",
                "</svg>",
            ]
        )
        return "\n".join(lines)

    all_points = [point for *_, points in track_points for point in points]
    xs = [float(point["x"]) for point in all_points]
    ys = [float(point["y"]) for point in all_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    margin = 72.0
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    def screen(point: dict) -> tuple[float, float]:
        x = width / 2.0 + (float(point["x"]) - center_x) * scale
        y = height / 2.0 - (float(point["y"]) - center_y) * scale
        return x, y

    grid_step = _nice_grid_step(max(span_x, span_y))
    left_m = center_x - width / 2.0 / scale
    right_m = center_x + width / 2.0 / scale
    bottom_m = center_y - height / 2.0 / scale
    top_m = center_y + height / 2.0 / scale
    grid_lines = []
    gx = math.floor(left_m / grid_step) * grid_step
    while gx <= right_m:
        x, _ = screen({"x": gx, "y": center_y})
        grid_lines.append(f'<line x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="{height}" stroke="#1f2925" stroke-width="1"/>')
        gx += grid_step
    gy = math.floor(bottom_m / grid_step) * grid_step
    while gy <= top_m:
        _, y = screen({"x": center_x, "y": gy})
        grid_lines.append(f'<line x1="0" y1="{y:.1f}" x2="{width}" y2="{y:.1f}" stroke="#1f2925" stroke-width="1"/>')
        gy += grid_step
    lines.extend(grid_lines)

    axis_x, _ = screen({"x": 0.0, "y": center_y})
    _, axis_y = screen({"x": center_x, "y": 0.0})
    if 0 <= axis_x <= width:
        lines.append(f'<line x1="{axis_x:.1f}" y1="0" x2="{axis_x:.1f}" y2="{height}" stroke="#53645b" stroke-width="1.5"/>')
    if 0 <= axis_y <= height:
        lines.append(f'<line x1="0" y1="{axis_y:.1f}" x2="{width}" y2="{axis_y:.1f}" stroke="#53645b" stroke-width="1.5"/>')

    for name, label, color, points in track_points:
        coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in (screen(point) for point in points))
        stroke_width = "3.0" if name == "filtered" else "2.2"
        opacity = "1.0" if name == "filtered" else "0.8"
        lines.append(
            f'<polyline points="{coords}" fill="none" stroke="{color}" '
            f'stroke-width="{stroke_width}" stroke-linejoin="round" stroke-linecap="round" opacity="{opacity}"/>'
        )
        sx, sy = screen(points[0])
        ex, ey = screen(points[-1])
        lines.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="5" fill="{color}"/>')
        lines.append(f'<rect x="{ex - 5:.1f}" y="{ey - 5:.1f}" width="10" height="10" fill="none" stroke="{color}" stroke-width="2"/>')

    lines.extend(_svg_header_and_legend(result, track_points, width))
    lines.append("</svg>")
    return "\n".join(lines)


def _new_artifact_dir(result: dict, output_root: str | Path | None) -> Path:
    root = Path(output_root).expanduser() if output_root else _default_artifact_root(result)
    root.mkdir(parents=True, exist_ok=True)
    bag_name = _safe_filename(Path(result["bag_path"]).name or "bag")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = root / f"{bag_name}_{stamp}"
    candidate = base
    index = 2
    while candidate.exists():
        candidate = root / f"{base.name}_{index:02d}"
        index += 1
    candidate.mkdir(parents=True)
    return candidate


def _default_artifact_root(result: dict) -> Path:
    env_root = os.environ.get("KMU26_BAG_ANALYSIS_ROOT")
    if env_root:
        return Path(env_root).expanduser()

    home_ws = Path.home() / "catkin_ws"
    if home_ws.exists():
        return home_ws / ARTIFACT_DIR_NAME

    bag_dir = Path(result.get("bag_path", "")).expanduser()
    for parent in [bag_dir, *bag_dir.parents]:
        if parent.name == "catkin_ws" or ((parent / "src").is_dir() and (parent / "install").is_dir()):
            return parent / ARTIFACT_DIR_NAME

    cwd = Path.cwd()
    for parent in [cwd, *cwd.parents]:
        if parent.name == "catkin_ws" or ((parent / "src").is_dir() and (parent / "install").is_dir()):
            return parent / ARTIFACT_DIR_NAME

    return home_ws / ARTIFACT_DIR_NAME


def _safe_filename(value: str) -> str:
    clean = "".join(char if char.isalnum() or char in {"-", "_"} else "_" for char in value)
    return clean.strip("_") or "bag"


def _nice_grid_step(span: float) -> float:
    if span <= 2:
        return 0.25
    if span <= 5:
        return 0.5
    if span <= 12:
        return 1.0
    if span <= 30:
        return 2.0
    return 5.0


def _svg_header_and_legend(result: dict, track_points: list, width: int) -> list[str]:
    assessment = result.get("assessment", {})
    status = str(assessment.get("status", "unknown")).upper()
    bag_name = _xml_escape(Path(result.get("bag_path", "bag")).name)
    lines = [
        '<rect x="20" y="18" width="430" height="96" rx="8" fill="#111614" stroke="#334039"/>',
        f'<text x="38" y="48" fill="#eef5f0" font-size="22" font-family="sans-serif">Localization Analysis: {status}</text>',
        f'<text x="38" y="76" fill="#9faea6" font-size="14" font-family="sans-serif">{bag_name}</text>',
    ]
    x = 38
    y = 96
    for _, label, color, _ in track_points:
        lines.append(f'<line x1="{x}" y1="{y}" x2="{x + 24}" y2="{y}" stroke="{color}" stroke-width="4"/>')
        lines.append(f'<text x="{x + 32}" y="{y + 5}" fill="#dce8e1" font-size="13" font-family="sans-serif">{_xml_escape(label)}</text>')
        x += 130
        if x > width - 160:
            x = 38
            y += 20
    return lines


def _xml_escape(value: Any) -> str:
    return (
        str(value)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _resolve_bag_paths(bag_path: str | Path) -> tuple[Path, Path]:
    path = Path(bag_path).expanduser()
    if path.is_file() and path.suffix == ".db3":
        return path.parent, path
    if not path.exists():
        raise RuntimeError(f"bag path does not exist: {path}")
    if not path.is_dir():
        raise RuntimeError(f"bag path is not a directory: {path}")
    if (path / "metadata.yaml").exists():
        return path, path

    db3_files = sorted(path.glob("*.db3"))
    if db3_files:
        return path, db3_files[0]

    if not db3_files:
        raise RuntimeError(f"no rosbag2 metadata.yaml or .db3 file was found in: {path}")
    return path, path


def _message_time(msg: Any, fallback_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        sec = getattr(stamp, "sec", 0)
        nanosec = getattr(stamp, "nanosec", 0)
        if sec or nanosec:
            return float(sec) + float(nanosec) * 1e-9
    return float(fallback_ns) * 1e-9


def _odom_point(msg: Any, t: float) -> dict | None:
    pose = getattr(getattr(msg, "pose", None), "pose", None)
    if pose is None:
        return None
    position = getattr(pose, "position", None)
    if position is None:
        return None
    point = _point_from_vector(position, t)
    if point is None:
        return None
    orientation = getattr(pose, "orientation", None)
    if orientation is not None:
        point["yaw"] = _yaw_from_quaternion(
            getattr(orientation, "x", 0.0),
            getattr(orientation, "y", 0.0),
            getattr(orientation, "z", 0.0),
            getattr(orientation, "w", 1.0),
        )
    return point


def _dvl_dr_point(msg: Any, t: float) -> dict | None:
    point = _point_from_vector(getattr(msg, "position", None), t)
    if point is None:
        return None
    pos_std = _finite_or_none(getattr(msg, "pos_std", None))
    if pos_std is not None:
        point["pos_std"] = pos_std
    yaw = _finite_or_none(getattr(msg, "yaw", None))
    if yaw is not None:
        point["yaw"] = yaw
    return point


def _point_from_vector(vector: Any, t: float) -> dict | None:
    if vector is None:
        return None
    x = _finite_or_none(getattr(vector, "x", None))
    y = _finite_or_none(getattr(vector, "y", None))
    z = _finite_or_none(getattr(vector, "z", None))
    if x is None or y is None:
        return None
    return {"t": t, "x": x, "y": y, "z": 0.0 if z is None else z}


def _track_metrics(points: list[dict]) -> dict:
    metrics = {
        "count": len(points),
        "duration_s": 0.0,
        "path_length_m": 0.0,
        "start_end_error_m": 0.0,
        "closure_ratio": None,
        "bbox_width_m": 0.0,
        "bbox_height_m": 0.0,
        "estimated_laps": 0.0,
        "mean_radius_m": 0.0,
        "max_radius_m": 0.0,
    }
    if not points:
        return metrics

    metrics["duration_s"] = max(0.0, points[-1]["t"] - points[0]["t"])
    xs = [point["x"] for point in points]
    ys = [point["y"] for point in points]
    metrics["bbox_width_m"] = max(xs) - min(xs)
    metrics["bbox_height_m"] = max(ys) - min(ys)

    path_length = 0.0
    for prev, curr in zip(points, points[1:]):
        path_length += math.hypot(curr["x"] - prev["x"], curr["y"] - prev["y"])
    metrics["path_length_m"] = path_length
    metrics["start_end_error_m"] = math.hypot(
        points[-1]["x"] - points[0]["x"],
        points[-1]["y"] - points[0]["y"],
    )
    if path_length > 1e-6:
        metrics["closure_ratio"] = metrics["start_end_error_m"] / path_length

    laps, mean_radius, max_radius = _estimate_laps(points)
    metrics["estimated_laps"] = laps
    metrics["mean_radius_m"] = mean_radius
    metrics["max_radius_m"] = max_radius
    return metrics


def _estimate_laps(points: list[dict]) -> tuple[float, float, float]:
    if len(points) < 4:
        return 0.0, 0.0, 0.0

    cx = sum(point["x"] for point in points) / len(points)
    cy = sum(point["y"] for point in points) / len(points)
    radii = [math.hypot(point["x"] - cx, point["y"] - cy) for point in points]
    max_radius = max(radii)
    mean_radius = sum(radii) / len(radii)
    if max_radius < 1e-3:
        return 0.0, mean_radius, max_radius

    angles = [
        math.atan2(point["y"] - cy, point["x"] - cx)
        for point, radius in zip(points, radii)
        if radius >= max_radius * 0.1
    ]
    if len(angles) < 4:
        return 0.0, mean_radius, max_radius

    total = 0.0
    previous = angles[0]
    for angle in angles[1:]:
        total += math.atan2(math.sin(angle - previous), math.cos(angle - previous))
        previous = angle
    return total / (2.0 * math.pi), mean_radius, max_radius


def _new_dvl_data_stats() -> dict:
    return {
        "count": 0,
        "valid_count": 0,
        "invalid_count": 0,
        "first_t": None,
        "last_t": None,
        "altitude": _new_stats(),
        "fom": _new_stats(),
        "speed": _new_stats(),
        "valid_beams": _new_stats(),
    }


def _update_dvl_data_stats(stats: dict, msg: Any, t: float) -> None:
    stats["count"] += 1
    stats["first_t"] = t if stats["first_t"] is None else stats["first_t"]
    stats["last_t"] = t

    if bool(getattr(msg, "velocity_valid", False)):
        stats["valid_count"] += 1
    else:
        stats["invalid_count"] += 1

    _update_stats(stats["altitude"], getattr(msg, "altitude", None))
    _update_stats(stats["fom"], getattr(msg, "fom", None))
    velocity = getattr(msg, "velocity", None)
    if velocity is not None:
        x = _finite_or_none(getattr(velocity, "x", None))
        y = _finite_or_none(getattr(velocity, "y", None))
        z = _finite_or_none(getattr(velocity, "z", None))
        if x is not None and y is not None and z is not None:
            _update_stats(stats["speed"], math.sqrt(x * x + y * y + z * z))

    beams = getattr(msg, "beams", [])
    valid_beams = sum(1 for beam in beams if bool(getattr(beam, "valid", False)))
    _update_stats(stats["valid_beams"], valid_beams)


def _finalize_dvl_data_stats(stats: dict) -> dict:
    count = stats["count"]
    duration = 0.0
    if stats["first_t"] is not None and stats["last_t"] is not None:
        duration = max(0.0, stats["last_t"] - stats["first_t"])
    return {
        "count": count,
        "duration_s": duration,
        "valid_count": stats["valid_count"],
        "invalid_count": stats["invalid_count"],
        "valid_rate": stats["valid_count"] / count if count else None,
        "altitude_m": _finalize_stats(stats["altitude"]),
        "fom": _finalize_stats(stats["fom"]),
        "speed_mps": _finalize_stats(stats["speed"]),
        "valid_beams": _finalize_stats(stats["valid_beams"]),
    }


def _new_dvl_twist_stats() -> dict:
    return {
        "count": 0,
        "first_t": None,
        "last_t": None,
        "speed": _new_stats(),
        "cov_x": _new_stats(),
        "cov_y": _new_stats(),
        "cov_z": _new_stats(),
    }


def _update_dvl_twist_stats(stats: dict, msg: Any, t: float) -> None:
    stats["count"] += 1
    stats["first_t"] = t if stats["first_t"] is None else stats["first_t"]
    stats["last_t"] = t

    twist = getattr(getattr(msg, "twist", None), "twist", None)
    linear = getattr(twist, "linear", None)
    if linear is not None:
        x = _finite_or_none(getattr(linear, "x", None))
        y = _finite_or_none(getattr(linear, "y", None))
        z = _finite_or_none(getattr(linear, "z", None))
        if x is not None and y is not None and z is not None:
            _update_stats(stats["speed"], math.sqrt(x * x + y * y + z * z))

    covariance = list(getattr(getattr(msg, "twist", None), "covariance", []))
    if len(covariance) >= 15:
        _update_stats(stats["cov_x"], covariance[0])
        _update_stats(stats["cov_y"], covariance[7])
        _update_stats(stats["cov_z"], covariance[14])


def _finalize_dvl_twist_stats(stats: dict) -> dict:
    duration = 0.0
    if stats["first_t"] is not None and stats["last_t"] is not None:
        duration = max(0.0, stats["last_t"] - stats["first_t"])
    return {
        "count": stats["count"],
        "duration_s": duration,
        "speed_mps": _finalize_stats(stats["speed"]),
        "covariance": {
            "x": _finalize_stats(stats["cov_x"]),
            "y": _finalize_stats(stats["cov_y"]),
            "z": _finalize_stats(stats["cov_z"]),
        },
    }


def _new_stats() -> dict:
    return {"count": 0, "sum": 0.0, "min": None, "max": None}


def _update_stats(stats: dict, value: Any) -> None:
    value = _finite_or_none(value)
    if value is None:
        return
    stats["count"] += 1
    stats["sum"] += value
    stats["min"] = value if stats["min"] is None else min(stats["min"], value)
    stats["max"] = value if stats["max"] is None else max(stats["max"], value)


def _finalize_stats(stats: dict) -> dict:
    count = stats["count"]
    return {
        "count": count,
        "mean": stats["sum"] / count if count else None,
        "min": stats["min"],
        "max": stats["max"],
    }


def _assessment(result: dict) -> dict:
    severity = 0
    notes: list[dict[str, str]] = []

    def add(level: str, message: str) -> None:
        nonlocal severity
        severity = max(severity, {"info": 0, "warn": 1, "bad": 2}[level])
        notes.append({"level": level, "message": message})

    filtered = result["odometry"]["filtered"]
    if filtered["count"] == 0:
        add("bad", "/odometry/filtered was not recorded.")
    else:
        path_length = filtered["path_length_m"]
        closure_ratio = filtered["closure_ratio"]
        laps = abs(filtered["estimated_laps"])
        add(
            "info",
            f"Filtered odom traveled {path_length:.2f} m, estimated laps {laps:.2f}.",
        )
        if path_length < 0.5:
            add("warn", "Filtered odom moved less than 0.5 m; loop quality cannot be judged.")
        elif closure_ratio is not None:
            if closure_ratio > 0.25:
                add("bad", f"Start/end closure error is high ({closure_ratio:.1%} of path length).")
            elif closure_ratio > 0.10:
                add("warn", f"Start/end closure error is moderate ({closure_ratio:.1%} of path length).")
        if path_length >= 3.0 and laps < 0.5:
            add("warn", "Trajectory does not look like a completed loop around its center.")

    dvl_data = result["dvl"]["data"]
    valid_rate = dvl_data["valid_rate"]
    if dvl_data["count"] == 0:
        add("bad", "/dvl/data was not recorded.")
    elif valid_rate is not None:
        if valid_rate < 0.85:
            add("bad", f"DVL valid rate is low ({valid_rate:.1%}).")
        elif valid_rate < 0.95:
            add("warn", f"DVL valid rate is below target ({valid_rate:.1%}).")

    fom_mean = dvl_data["fom"]["mean"]
    if fom_mean is not None and fom_mean > 0.05:
        add("warn", f"DVL mean FOM is high ({fom_mean:.3f}).")

    altitude_min = dvl_data["altitude_m"]["min"]
    if altitude_min is not None and altitude_min < 0.05:
        add("warn", f"DVL altitude dropped near zero ({altitude_min:.3f} m).")

    status = ["good", "warn", "bad"][severity]
    return {"status": status, "notes": notes}


def _decimate(points: list[dict], max_samples: int) -> list[dict]:
    if max_samples <= 0 or len(points) <= max_samples:
        return points
    step = (len(points) - 1) / float(max_samples - 1)
    indexes = {round(i * step) for i in range(max_samples)}
    indexes.add(0)
    indexes.add(len(points) - 1)
    return [points[index] for index in sorted(indexes)]


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_or_none(value: Any) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a localization rosbag.")
    parser.add_argument("bag", help="rosbag2 directory or .db3 file")
    parser.add_argument("-o", "--output", default="", help="JSON output path")
    parser.add_argument("--max-samples", default=600, type=int)
    parser.add_argument("--pretty", action="store_true", help="print indented JSON")
    args = parser.parse_args()

    result = analyze_bag(args.bag, max_samples=args.max_samples)
    if args.output:
        result["report_path"] = write_analysis_report(result, args.output)

    indent = 2 if args.pretty else None
    try:
        print(json.dumps(result, ensure_ascii=False, indent=indent))
    except BrokenPipeError:
        return


if __name__ == "__main__":
    main()
