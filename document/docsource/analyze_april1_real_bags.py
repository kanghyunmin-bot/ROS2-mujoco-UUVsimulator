from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ModuleNotFoundError:
    rosbag2_py = None
    deserialize_message = None
    get_message = None
try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
except ModuleNotFoundError:
    AnyReader = None
    Stores = None
    get_typestore = None


DEFAULT_ROOT = Path("real_robot_ros_bag/extracted_2026_04_01")
DEFAULT_OUT = Path("document/docsource/real_bag_2026_04_01_analysis")


TARGET_TOPICS = {
    "/mavros/imu/data",
    "/mavros/imu/data_raw",
    "/camera/camera/gyro/sample",
    "/camera/camera/accel/sample",
    "/dvl/twist",
    "/dvl/odometry",
    "/depth/pose",
    "/depth",
    "/mavros/imu/atm_pressure",
    "/mavros/imu/static_pressure",
    "/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/odometry/filtered",
    "/mavros/rc/in",
    "/mavros/rc/out",
    "/mavros/rc/override",
    "/joy",
    "/mavros/state",
    "/mavros/vfr_hud",
    "/mavros/nav_controller_output/output",
    "/tf",
    "/tf_static",
}


def quat_to_euler_xyzw(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def stamp_to_ns(stamp: Any) -> int:
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    if values.size == 0 or samples <= 1:
        return values.copy()
    samples = min(samples, max(1, values.size // 2))
    kernel = np.ones(samples, dtype=float) / float(samples)
    padded = np.pad(values.astype(float), (samples // 2, samples - 1 - samples // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def robust_highpass_std(t: np.ndarray, y: np.ndarray, window_s: float = 2.0) -> float | None:
    if t.size < 8 or y.size < 8:
        return None
    finite = np.isfinite(t) & np.isfinite(y)
    t = t[finite]
    y = y[finite]
    if t.size < 8:
        return None
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return None
    samples = max(3, int(round(window_s / float(np.median(dt)))))
    trend = moving_average(y, samples)
    residual = y - trend
    q1, q3 = np.percentile(residual, [25, 75])
    iqr = q3 - q1
    if iqr > 1e-12:
        residual = residual[(residual >= q1 - 3.0 * iqr) & (residual <= q3 + 3.0 * iqr)]
    if residual.size < 3:
        return None
    return float(np.std(residual))


def scalar_stats(t: np.ndarray, y: np.ndarray) -> dict[str, Any]:
    out: dict[str, Any] = {"count": int(y.size)}
    if y.size == 0:
        return out
    finite = np.isfinite(y)
    if t.size == y.size:
        finite &= np.isfinite(t)
    y = y[finite]
    t = t[finite] if t.size == finite.size else np.array([], dtype=float)
    out["count"] = int(y.size)
    if y.size == 0:
        return out
    out.update(
        {
            "mean": float(np.mean(y)),
            "std": float(np.std(y)),
            "rms": float(np.sqrt(np.mean(y * y))),
            "min": float(np.min(y)),
            "p01": float(np.percentile(y, 1)),
            "p05": float(np.percentile(y, 5)),
            "p50": float(np.percentile(y, 50)),
            "p95": float(np.percentile(y, 95)),
            "p99": float(np.percentile(y, 99)),
            "max": float(np.max(y)),
            "range": float(np.max(y) - np.min(y)),
            "highpass_std_2s": robust_highpass_std(t, y, 2.0) if t.size == y.size else None,
        }
    )
    if t.size >= 3:
        dt = np.diff(t)
        dt = dt[np.isfinite(dt) & (dt > 0.0)]
        if dt.size:
            median_dt = float(np.median(dt))
            out.update(
                {
                    "duration_s": float(t[-1] - t[0]),
                    "median_rate_hz": float(1.0 / median_dt),
                    "mean_rate_hz": float((t.size - 1) / max(t[-1] - t[0], 1e-9)),
                    "median_dt_s": median_dt,
                    "dt_jitter_std_s": float(np.std(dt)),
                    "dt_p95_s": float(np.percentile(dt, 95)),
                    "max_dt_s": float(np.max(dt)),
                    "gap_count_gt_2x_median": int(np.sum(dt > 2.0 * median_dt + 1e-9)),
                    "gap_count_gt_0p2s": int(np.sum(dt > 0.2)),
                    "gap_count_gt_0p5s": int(np.sum(dt > 0.5)),
                }
            )
    return out


def vector_stats(t: np.ndarray, values: np.ndarray, names: tuple[str, str, str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if values.size == 0:
        for name in names:
            out[name] = {"count": 0}
        out["norm"] = {"count": 0}
        return out
    for idx, name in enumerate(names):
        out[name] = scalar_stats(t, values[:, idx])
    out["norm"] = scalar_stats(t, np.linalg.norm(values, axis=1))
    return out


def covariance_summary(values: np.ndarray) -> dict[str, Any]:
    if values.size == 0:
        return {"count": 0}
    out = {
        "count": int(values.shape[0]),
        "mean_diag": [float(v) for v in np.mean(values, axis=0)],
        "median_diag": [float(v) for v in np.median(values, axis=0)],
        "min_diag": [float(v) for v in np.min(values, axis=0)],
        "max_diag": [float(v) for v in np.max(values, axis=0)],
    }
    rounded = np.round(values, decimals=12)
    unique_rows = np.unique(rounded, axis=0)
    out["unique_diag_count"] = int(unique_rows.shape[0])
    out["first_unique_diags"] = [[float(v) for v in row] for row in unique_rows[:8]]
    return out


@dataclass
class BagData:
    name: str
    db_path: Path
    topics: dict[str, str]
    topic_counts: dict[str, int]
    t0_ns: int
    t1_ns: int
    frames: dict[str, Counter[str]] = field(default_factory=lambda: defaultdict(Counter))
    header_lag_s: dict[str, list[float]] = field(default_factory=lambda: defaultdict(list))
    tf_edges: Counter[str] = field(default_factory=Counter)
    static_tf_edges: Counter[str] = field(default_factory=Counter)
    states: Counter[str] = field(default_factory=Counter)
    bool_states: dict[str, Counter[str]] = field(default_factory=lambda: defaultdict(Counter))
    series_t: dict[str, list[float]] = field(default_factory=lambda: defaultdict(list))
    series_v: dict[str, list[Any]] = field(default_factory=lambda: defaultdict(list))
    covariances: dict[str, list[Any]] = field(default_factory=lambda: defaultdict(list))
    skipped_topics: dict[str, str] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        return max(0.0, (self.t1_ns - self.t0_ns) * 1e-9)

    def add_scalar(self, key: str, t: float, value: float) -> None:
        self.series_t[key].append(t)
        self.series_v[key].append(float(value))

    def add_vector(self, key: str, t: float, values: list[float] | tuple[float, ...]) -> None:
        self.series_t[key].append(t)
        self.series_v[key].append([float(v) for v in values])

    def array(self, key: str, dims: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        t = np.asarray(self.series_t.get(key, []), dtype=float)
        v = np.asarray(self.series_v.get(key, []), dtype=float)
        if dims is not None:
            if v.size:
                v = v.reshape((-1, dims))
            else:
                v = np.empty((0, dims), dtype=float)
        return t, v

    def cov_array(self, key: str) -> np.ndarray:
        v = np.asarray(self.covariances.get(key, []), dtype=float)
        if v.size:
            return v.reshape((-1, 3))
        return np.empty((0, 3), dtype=float)


def discover_bags(root: Path) -> list[Path]:
    if root.is_file() and root.suffix == ".db3":
        return [root]
    return sorted(root.glob("bag_*/**/*.db3"))


def resolve_bag_db3(path: Path) -> Path:
    path = Path(path)
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.db3")) or sorted(path.glob("**/*.db3"))
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"No .db3 rosbag storage found at {path}")


def open_rosbag2_reader(path: Path) -> tuple[Any, Path]:
    if rosbag2_py is None:
        raise ModuleNotFoundError("rosbag2_py is required to read ROS 2 bags")
    db_path = resolve_bag_db3(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(db_path.parent), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader, db_path


def read_bag_metadata(db_path: Path) -> tuple[dict[str, str], dict[str, int], int, int]:
    reader, _ = open_rosbag2_reader(db_path)
    topics = {str(topic.name): str(topic.type) for topic in reader.get_all_topics_and_types()}
    counts = {name: 0 for name in topics}
    t0_ns: int | None = None
    t1_ns: int | None = None
    while reader.has_next():
        topic, _raw, timestamp_ns = reader.read_next()
        topic = str(topic)
        timestamp_ns = int(timestamp_ns)
        counts[topic] = counts.get(topic, 0) + 1
        if t0_ns is None or timestamp_ns < t0_ns:
            t0_ns = timestamp_ns
        if t1_ns is None or timestamp_ns > t1_ns:
            t1_ns = timestamp_ns
    return topics, counts, int(t0_ns or 0), int(t1_ns or 0)


def read_string_topic_events(db_path: Path, topic: str) -> list[tuple[float, str]]:
    if deserialize_message is None or get_message is None:
        raise ModuleNotFoundError("rclpy.serialization.deserialize_message is required")
    topics, _, t0_ns, _ = read_bag_metadata(db_path)
    type_name = topics.get(topic)
    if type_name is None:
        return []
    msg_cls = get_message(type_name)
    reader, _ = open_rosbag2_reader(db_path)
    events: list[tuple[float, str]] = []
    while reader.has_next():
        msg_topic, raw, timestamp_ns = reader.read_next()
        if msg_topic != topic:
            continue
        msg = deserialize_message(raw, msg_cls)
        events.append(((int(timestamp_ns) - int(t0_ns)) * 1.0e-9, str(getattr(msg, "data", ""))))
    return events


def first_string_time_in_bag(db_path: Path, topic: str, value: str) -> float | None:
    for t, event_value in read_string_topic_events(db_path, topic):
        if event_value == value:
            return float(t)
    return None


def maybe_record_header(data: BagData, topic: str, msg: Any, timestamp_ns: int) -> None:
    header = getattr(msg, "header", None)
    if header is None:
        return
    frame_id = str(getattr(header, "frame_id", ""))
    data.frames[topic][frame_id] += 1
    msg_stamp_ns = stamp_to_ns(getattr(header, "stamp", None))
    if msg_stamp_ns > 0:
        data.header_lag_s[topic].append((timestamp_ns - msg_stamp_ns) * 1e-9)


def handle_imu(data: BagData, topic: str, msg: Any, t: float) -> None:
    q = msg.orientation
    roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
    data.add_vector(f"{topic}:rpy_rad", t, [roll, pitch, yaw])
    data.add_vector(
        f"{topic}:gyro_rad_s",
        t,
        [float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z)],
    )
    data.add_vector(
        f"{topic}:accel_m_s2",
        t,
        [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y), float(msg.linear_acceleration.z)],
    )
    data.covariances[f"{topic}:orientation_cov"].append(
        [float(msg.orientation_covariance[i]) for i in (0, 4, 8)]
    )
    data.covariances[f"{topic}:gyro_cov"].append(
        [float(msg.angular_velocity_covariance[i]) for i in (0, 4, 8)]
    )
    data.covariances[f"{topic}:accel_cov"].append(
        [float(msg.linear_acceleration_covariance[i]) for i in (0, 4, 8)]
    )


def handle_dvl_twist(data: BagData, topic: str, msg: Any, t: float) -> None:
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular
    data.add_vector(f"{topic}:linear_m_s", t, [float(lin.x), float(lin.y), float(lin.z)])
    data.add_vector(f"{topic}:angular_rad_s", t, [float(ang.x), float(ang.y), float(ang.z)])
    data.covariances[f"{topic}:linear_cov"].append([float(msg.twist.covariance[i]) for i in (0, 7, 14)])
    data.covariances[f"{topic}:angular_cov"].append([float(msg.twist.covariance[i]) for i in (21, 28, 35)])


def handle_depth_pose(data: BagData, topic: str, msg: Any, t: float) -> None:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
    z = float(p.z)
    data.add_scalar(f"{topic}:z_m", t, z)
    data.add_scalar(f"{topic}:depth_positive_m", t, -z)
    data.add_vector(f"{topic}:xyz_m", t, [float(p.x), float(p.y), z])
    data.add_vector(f"{topic}:rpy_rad", t, [roll, pitch, yaw])
    data.covariances[f"{topic}:position_cov"].append(
        [float(msg.pose.covariance[i]) for i in (0, 7, 14)]
    )


def handle_depth_scalar(data: BagData, topic: str, msg: Any, t: float) -> None:
    depth = float(getattr(msg, "data", 0.0))
    data.add_scalar(f"{topic}:depth_positive_m", t, depth)


def handle_pressure(data: BagData, topic: str, msg: Any, t: float) -> None:
    data.add_scalar(f"{topic}:pressure_pa", t, float(msg.fluid_pressure))
    data.add_scalar(f"{topic}:variance", t, float(getattr(msg, "variance", 0.0)))


def handle_odom(data: BagData, topic: str, msg: Any, t: float) -> None:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular
    data.add_vector(f"{topic}:xyz_m", t, [float(p.x), float(p.y), float(p.z)])
    data.add_vector(f"{topic}:rpy_rad", t, [roll, pitch, yaw])
    data.add_vector(f"{topic}:linear_m_s", t, [float(lin.x), float(lin.y), float(lin.z)])
    data.add_vector(f"{topic}:angular_rad_s", t, [float(ang.x), float(ang.y), float(ang.z)])
    data.covariances[f"{topic}:pose_position_cov"].append(
        [float(msg.pose.covariance[i]) for i in (0, 7, 14)]
    )
    data.covariances[f"{topic}:twist_linear_cov"].append(
        [float(msg.twist.covariance[i]) for i in (0, 7, 14)]
    )


def handle_pose(data: BagData, topic: str, msg: Any, t: float) -> None:
    p = msg.pose.position
    q = msg.pose.orientation
    roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
    data.add_vector(f"{topic}:xyz_m", t, [float(p.x), float(p.y), float(p.z)])
    data.add_vector(f"{topic}:rpy_rad", t, [roll, pitch, yaw])


def handle_twist(data: BagData, topic: str, msg: Any, t: float) -> None:
    lin = msg.twist.linear
    ang = msg.twist.angular
    data.add_vector(f"{topic}:linear_m_s", t, [float(lin.x), float(lin.y), float(lin.z)])
    data.add_vector(f"{topic}:angular_rad_s", t, [float(ang.x), float(ang.y), float(ang.z)])


def handle_rc(data: BagData, topic: str, msg: Any, t: float) -> None:
    channels = [float(v) for v in getattr(msg, "channels", [])]
    if channels:
        data.add_vector(f"{topic}:channels", t, channels)
    for name in ("rssi", "chancount"):
        if hasattr(msg, name):
            data.add_scalar(f"{topic}:{name}", t, float(getattr(msg, name)))


def handle_joy(data: BagData, topic: str, msg: Any, t: float) -> None:
    axes = [float(v) for v in getattr(msg, "axes", [])]
    buttons = [float(v) for v in getattr(msg, "buttons", [])]
    if axes:
        data.add_vector(f"{topic}:axes", t, axes)
    if buttons:
        data.add_vector(f"{topic}:buttons", t, buttons)


def handle_state(data: BagData, topic: str, msg: Any, t: float) -> None:
    del t
    mode = str(getattr(msg, "mode", ""))
    data.states[mode] += 1
    for name in ("connected", "armed", "guided", "manual_input"):
        if hasattr(msg, name):
            data.bool_states[name][str(bool(getattr(msg, name)))] += 1
    maybe_frame = getattr(getattr(msg, "header", None), "frame_id", None)
    if maybe_frame:
        data.frames[topic][str(maybe_frame)] += 1


def handle_vfr_hud(data: BagData, topic: str, msg: Any, t: float) -> None:
    for name in ("airspeed", "groundspeed", "heading", "throttle", "altitude", "climb"):
        if hasattr(msg, name):
            data.add_scalar(f"{topic}:{name}", t, float(getattr(msg, name)))


def handle_nav_output(data: BagData, topic: str, msg: Any, t: float) -> None:
    for name in (
        "nav_roll",
        "nav_pitch",
        "nav_bearing",
        "target_bearing",
        "wp_dist",
        "alt_error",
        "aspd_error",
        "xtrack_error",
    ):
        if hasattr(msg, name):
            data.add_scalar(f"{topic}:{name}", t, float(getattr(msg, name)))


def handle_tf(data: BagData, topic: str, msg: Any, t: float) -> None:
    counter = data.static_tf_edges if topic == "/tf_static" else data.tf_edges
    for transform in msg.transforms:
        parent = str(transform.header.frame_id)
        child = str(transform.child_frame_id)
        edge = f"{parent}->{child}"
        counter[edge] += 1
        data.frames[topic][parent] += 1
        data.header_lag_s[topic].append(0.0)
        if (parent, child) in {("odom", "base_link"), ("odom", "fcu_link"), ("map", "odom")}:
            p = transform.transform.translation
            q = transform.transform.rotation
            roll, pitch, yaw = quat_to_euler_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
            key = f"{topic}:{edge}"
            data.add_vector(f"{key}:xyz_m", t, [float(p.x), float(p.y), float(p.z)])
            data.add_vector(f"{key}:rpy_rad", t, [roll, pitch, yaw])


def handle_message(data: BagData, topic: str, msg: Any, t: float) -> None:
    if topic in {"/mavros/imu/data", "/mavros/imu/data_raw", "/camera/camera/gyro/sample", "/camera/camera/accel/sample"}:
        handle_imu(data, topic, msg, t)
    elif topic == "/dvl/twist":
        handle_dvl_twist(data, topic, msg, t)
    elif topic == "/dvl/odometry":
        handle_odom(data, topic, msg, t)
    elif topic == "/depth/pose":
        handle_depth_pose(data, topic, msg, t)
    elif topic == "/depth":
        handle_depth_scalar(data, topic, msg, t)
    elif topic in {"/mavros/imu/atm_pressure", "/mavros/imu/static_pressure"}:
        handle_pressure(data, topic, msg, t)
    elif topic in {"/mavros/local_position/odom", "/odometry/filtered"}:
        handle_odom(data, topic, msg, t)
    elif topic == "/mavros/local_position/pose":
        handle_pose(data, topic, msg, t)
    elif topic == "/mavros/local_position/velocity_local":
        handle_twist(data, topic, msg, t)
    elif topic in {"/mavros/rc/in", "/mavros/rc/out", "/mavros/rc/override"}:
        handle_rc(data, topic, msg, t)
    elif topic == "/joy":
        handle_joy(data, topic, msg, t)
    elif topic == "/mavros/state":
        handle_state(data, topic, msg, t)
    elif topic == "/mavros/vfr_hud":
        handle_vfr_hud(data, topic, msg, t)
    elif topic == "/mavros/nav_controller_output/output":
        handle_nav_output(data, topic, msg, t)
    elif topic in {"/tf", "/tf_static"}:
        handle_tf(data, topic, msg, t)


def align4(offset: int) -> int:
    return (offset + 3) & ~3


def parse_u16_values(raw: bytes, offset: int, count: int) -> list[float]:
    needed = offset + count * 2
    if len(raw) < needed:
        return []
    return [float(int.from_bytes(raw[offset + 2 * idx : offset + 2 * idx + 2], "little")) for idx in range(count)]


def decode_override_rc_in(raw: bytes) -> SimpleNamespace:
    # CDR header is 4 bytes, followed by uint16[18] channels.
    return SimpleNamespace(channels=parse_u16_values(raw, 4, 18))


def decode_rc_out(raw: bytes) -> SimpleNamespace:
    # CDR header + std_msgs/Header + uint16[] channels.
    offset = 4 + 8
    if len(raw) < offset + 4:
        return SimpleNamespace(channels=[])
    frame_len = int.from_bytes(raw[offset : offset + 4], "little")
    offset = align4(offset + 4 + frame_len)
    if len(raw) < offset + 4:
        return SimpleNamespace(channels=[])
    channel_count = int.from_bytes(raw[offset : offset + 4], "little")
    offset += 4
    return SimpleNamespace(channels=parse_u16_values(raw, offset, channel_count))


def read_bag_with_rosbags(db_path: Path) -> BagData:
    if AnyReader is None or Stores is None or get_typestore is None:
        raise ModuleNotFoundError("Neither rclpy nor rosbags is available for rosbag decoding")
    resolved_db_path = resolve_bag_db3(db_path)
    topics, counts, t0_ns, t1_ns = read_bag_metadata(resolved_db_path)
    data = BagData(name=resolved_db_path.parent.name, db_path=resolved_db_path, topics=topics, topic_counts=counts, t0_ns=t0_ns, t1_ns=t1_ns)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([resolved_db_path.parent], default_typestore=typestore) as reader:
        target_connections = [conn for conn in reader.connections if conn.topic in TARGET_TOPICS]
        for conn, timestamp_ns, raw in reader.messages(connections=target_connections):
            timestamp_ns = int(timestamp_ns)
            topic = str(conn.topic)
            t = (timestamp_ns - t0_ns) * 1e-9
            try:
                if topic == "/mavros/rc/override":
                    msg = decode_override_rc_in(bytes(raw))
                elif topic == "/mavros/rc/out":
                    msg = decode_rc_out(bytes(raw))
                else:
                    msg = reader.deserialize(raw, conn.msgtype)
                    maybe_record_header(data, topic, msg, timestamp_ns)
                handle_message(data, topic, msg, t)
            except Exception as exc:
                data.skipped_topics[topic] = f"{conn.msgtype}: {exc}"
    return data


def read_bag(db_path: Path) -> BagData:
    if rosbag2_py is None or deserialize_message is None or get_message is None:
        return read_bag_with_rosbags(db_path)
    resolved_db_path = resolve_bag_db3(db_path)
    topics, counts, t0_ns, t1_ns = read_bag_metadata(resolved_db_path)
    data = BagData(name=resolved_db_path.parent.name, db_path=resolved_db_path, topics=topics, topic_counts=counts, t0_ns=t0_ns, t1_ns=t1_ns)
    target_msg_types: dict[str, Any] = {}
    for name, type_name in sorted(topics.items()):
        if name not in TARGET_TOPICS:
            continue
        try:
            target_msg_types[name] = get_message(type_name)
        except Exception as exc:
            data.skipped_topics[name] = f"{type_name}: {exc}"

    reader, _ = open_rosbag2_reader(resolved_db_path)
    while reader.has_next():
        name, raw, timestamp_ns = reader.read_next()
        msg_cls = target_msg_types.get(str(name))
        if msg_cls is None:
            continue
        timestamp_ns = int(timestamp_ns)
        t = (timestamp_ns - t0_ns) * 1e-9
        msg = deserialize_message(raw, msg_cls)
        maybe_record_header(data, str(name), msg, timestamp_ns)
        handle_message(data, str(name), msg, t)
    return data


def summarize_bag(data: BagData) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "name": data.name,
        "db_path": str(data.db_path),
        "duration_s": data.duration_s,
        "message_count": int(sum(data.topic_counts.values())),
        "topic_count": len(data.topics),
        "topic_counts": data.topic_counts,
        "topic_types": data.topics,
        "frames": {topic: dict(counter) for topic, counter in data.frames.items()},
        "tf_edges_top": data.tf_edges.most_common(30),
        "tf_static_edges": data.static_tf_edges.most_common(50),
        "state_modes": dict(data.states),
        "state_flags": {key: dict(value) for key, value in data.bool_states.items()},
        "skipped_topics": data.skipped_topics,
        "rates": {},
        "signals": {},
        "covariances": {},
        "header_lag_s": {},
        "derived": {},
    }

    for topic, count in sorted(data.topic_counts.items()):
        if count <= 0:
            continue
        summary["rates"][topic] = {
            "count": int(count),
            "mean_rate_hz_over_bag": float(count / max(data.duration_s, 1e-9)),
        }

    vector_names = {
        ":rpy_rad": ("roll", "pitch", "yaw"),
        ":gyro_rad_s": ("x", "y", "z"),
        ":accel_m_s2": ("x", "y", "z"),
        ":linear_m_s": ("x", "y", "z"),
        ":angular_rad_s": ("x", "y", "z"),
        ":xyz_m": ("x", "y", "z"),
    }
    for key in sorted(data.series_v):
        t = np.asarray(data.series_t.get(key, []), dtype=float)
        raw = np.asarray(data.series_v.get(key, []), dtype=float)
        if raw.size == 0:
            continue
        suffix = next((suffix for suffix in vector_names if key.endswith(suffix)), None)
        if suffix is not None:
            values = raw.reshape((-1, 3))
            summary["signals"][key] = vector_stats(t, values, vector_names[suffix])
        elif raw.ndim == 2:
            summary["signals"][key] = {
                f"idx_{idx}": scalar_stats(t, raw[:, idx])
                for idx in range(raw.shape[1])
            }
        else:
            summary["signals"][key] = scalar_stats(t, raw.reshape(-1))

    for key in sorted(data.covariances):
        summary["covariances"][key] = covariance_summary(data.cov_array(key))

    for topic, values in sorted(data.header_lag_s.items()):
        arr = np.asarray(values, dtype=float)
        summary["header_lag_s"][topic] = scalar_stats(np.arange(arr.size, dtype=float), arr)

    add_derived_metrics(data, summary)
    return summary


def finite_mask(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    mask = np.isfinite(t)
    if v.ndim == 1:
        mask &= np.isfinite(v)
    else:
        mask &= np.all(np.isfinite(v), axis=1)
    return mask


def scalar_interp_compare(
    ref_t: np.ndarray,
    ref_y: np.ndarray,
    other_t: np.ndarray,
    other_y: np.ndarray,
    *,
    residual_sign: float = 1.0,
) -> dict[str, Any]:
    ref_y = ref_y.reshape(-1)
    other_y = other_y.reshape(-1)
    if ref_t.size < 3 or other_t.size < 3 or ref_y.size != ref_t.size or other_y.size != other_t.size:
        return {"count": 0}
    start = max(float(ref_t[0]), float(other_t[0]))
    end = min(float(ref_t[-1]), float(other_t[-1]))
    mask = (ref_t >= start) & (ref_t <= end)
    if np.sum(mask) < 3:
        return {"count": 0}
    ref_t2 = ref_t[mask]
    ref_y2 = ref_y[mask]
    other_interp = np.interp(ref_t2, other_t, other_y)
    residual = residual_sign * (ref_y2 - other_interp)
    corr = None
    if np.std(ref_y2) > 1e-12 and np.std(other_interp) > 1e-12:
        corr = float(np.corrcoef(ref_y2, other_interp)[0, 1])
    return {
        "count": int(ref_t2.size),
        "overlap_s": float(end - start),
        "ref": scalar_stats(ref_t2, ref_y2),
        "other_interp": scalar_stats(ref_t2, other_interp),
        "residual": scalar_stats(ref_t2, residual),
        "correlation": corr,
    }


def vector_interp_compare(
    ref_t: np.ndarray,
    ref_v: np.ndarray,
    other_t: np.ndarray,
    other_v: np.ndarray,
    *,
    remove_median_offset: bool = False,
) -> dict[str, Any]:
    if ref_t.size < 3 or other_t.size < 3 or ref_v.size == 0 or other_v.size == 0:
        return {"count": 0}
    start = max(float(ref_t[0]), float(other_t[0]))
    end = min(float(ref_t[-1]), float(other_t[-1]))
    mask = (ref_t >= start) & (ref_t <= end)
    if np.sum(mask) < 3:
        return {"count": 0}
    ref_t2 = ref_t[mask]
    ref_v2 = ref_v[mask]
    other_interp = np.column_stack([np.interp(ref_t2, other_t, other_v[:, idx]) for idx in range(ref_v2.shape[1])])
    residual = ref_v2 - other_interp
    offset = np.zeros(ref_v2.shape[1], dtype=float)
    if remove_median_offset:
        offset = np.median(residual, axis=0)
        residual = residual - offset
    return {
        "count": int(ref_t2.size),
        "overlap_s": float(end - start),
        "removed_median_offset": [float(v) for v in offset],
        "residual": vector_stats(ref_t2, residual, ("x", "y", "z")),
    }


def add_derived_metrics(data: BagData, summary: dict[str, Any]) -> None:
    dvl_t, dvl_v = data.array("/dvl/twist:linear_m_s", 3)
    if dvl_v.size:
        speed = np.linalg.norm(dvl_v, axis=1)
        summary["derived"]["dvl_speed_m_s"] = scalar_stats(dvl_t, speed)
        low_mask = speed <= max(0.04, float(np.percentile(speed, 25)))
        if np.sum(low_mask) >= 10:
            summary["derived"]["dvl_low_speed_sample_count"] = int(np.sum(low_mask))
            summary["derived"]["dvl_low_speed_velocity_stats"] = vector_stats(dvl_t[low_mask], dvl_v[low_mask], ("x", "y", "z"))

    depth_t, depth = data.array("/depth/pose:depth_positive_m")
    if depth.size:
        summary["derived"]["depth_positive_m"] = scalar_stats(depth_t, depth.reshape(-1))

    atm_t, atm = data.array("/mavros/imu/atm_pressure:pressure_pa")
    static_t, static_p = data.array("/mavros/imu/static_pressure:pressure_pa")
    if atm.size and depth.size:
        atm = atm.reshape(-1)
        interp_depth = np.interp(atm_t, depth_t, depth.reshape(-1))
        rho_g_est = None
        if np.ptp(interp_depth) > 0.05:
            coeff = np.polyfit(interp_depth, atm, 1)
            rho_g_est = float(coeff[0])
        summary["derived"]["atm_pressure_vs_depth"] = {
            "pressure_pa": scalar_stats(atm_t, atm),
            "estimated_rho_g_pa_per_m": rho_g_est,
        }
    if static_p.size:
        static_p = static_p.reshape(-1)
        summary["derived"]["static_pressure_pa"] = scalar_stats(static_t, static_p)
        if depth.size:
            depth_flat = depth.reshape(-1)
            interp_depth = np.interp(static_t, depth_t, depth_flat)
            rho_g_est = None
            intercept = None
            residual_stats = {"count": 0}
            corr = None
            if np.ptp(interp_depth) > 0.05:
                coeff = np.polyfit(interp_depth, static_p, 1)
                rho_g_est = float(coeff[0])
                intercept = float(coeff[1])
                residual_stats = scalar_stats(static_t, static_p - np.polyval(coeff, interp_depth))
                if np.std(interp_depth) > 1e-12 and np.std(static_p) > 1e-12:
                    corr = float(np.corrcoef(interp_depth, static_p)[0, 1])
            summary["derived"]["static_pressure_vs_depth"] = {
                "estimated_rho_g_pa_per_m": rho_g_est,
                "estimated_surface_pressure_pa": intercept,
                "fit_residual_pa": residual_stats,
                "correlation": corr,
            }

    odom_t, odom_xyz = data.array("/odometry/filtered:xyz_m", 3)
    local_t, local_xyz = data.array("/mavros/local_position/odom:xyz_m", 3)
    tf_t, tf_xyz = data.array("/tf:odom->base_link:xyz_m", 3)
    if depth.size and odom_xyz.size:
        summary["derived"]["depth_vs_negative_odometry_filtered_z"] = scalar_interp_compare(
            depth_t,
            depth.reshape(-1),
            odom_t,
            -odom_xyz[:, 2],
        )
    if depth.size and local_xyz.size:
        summary["derived"]["depth_vs_negative_mavros_local_odom_z"] = scalar_interp_compare(
            depth_t,
            depth.reshape(-1),
            local_t,
            -local_xyz[:, 2],
        )
    if odom_xyz.size and local_xyz.size:
        summary["derived"]["local_odom_vs_odometry_filtered_xyz_offset_removed"] = vector_interp_compare(
            local_t,
            local_xyz,
            odom_t,
            odom_xyz,
            remove_median_offset=True,
        )
    if odom_xyz.size and tf_xyz.size:
        summary["derived"]["tf_vs_odometry_filtered_xyz"] = vector_interp_compare(
            tf_t,
            tf_xyz,
            odom_t,
            odom_xyz,
            remove_median_offset=False,
        )

    rc_t, rc = data.array("/mavros/rc/override:channels")
    if rc.size:
        neutral = np.nanmedian(rc, axis=0)
        deviation = np.abs(rc - neutral)
        active = np.any(deviation > 30.0, axis=1)
        summary["derived"]["rc_override_channels"] = {
            "channel_count": int(rc.shape[1]),
            "median": [float(v) for v in neutral],
            "p05": [float(v) for v in np.percentile(rc, 5, axis=0)],
            "p95": [float(v) for v in np.percentile(rc, 95, axis=0)],
            "active_fraction_gt_30us_from_median": float(np.mean(active)),
        }

    joy_t, joy_axes = data.array("/joy:axes")
    if joy_axes.size:
        summary["derived"]["joy_axes"] = {
            "axis_count": int(joy_axes.shape[1]),
            "mean_abs": [float(v) for v in np.mean(np.abs(joy_axes), axis=0)],
            "p95_abs": [float(v) for v in np.percentile(np.abs(joy_axes), 95, axis=0)],
        }

    for imu_topic in ("/mavros/imu/data", "/mavros/imu/data_raw", "/camera/camera/gyro/sample", "/camera/camera/accel/sample"):
        gyro_t, gyro = data.array(f"{imu_topic}:gyro_rad_s", 3)
        accel_t, accel = data.array(f"{imu_topic}:accel_m_s2", 3)
        if gyro.size:
            summary["derived"][f"{imu_topic}:gyro_noise_estimate"] = {
                axis: robust_highpass_std(gyro_t, gyro[:, idx], 2.0)
                for idx, axis in enumerate(("x", "y", "z"))
            }
        if accel.size:
            summary["derived"][f"{imu_topic}:accel_noise_estimate"] = {
                axis: robust_highpass_std(accel_t, accel[:, idx], 2.0)
                for idx, axis in enumerate(("x", "y", "z"))
            }


def plot_bag(data: BagData, out_dir: Path) -> list[str]:
    paths: list[str] = []
    out_dir.mkdir(parents=True, exist_ok=True)

    depth_t, depth = data.array("/depth/pose:depth_positive_m")
    dvl_t, dvl_v = data.array("/dvl/twist:linear_m_s", 3)
    rc_t, rc = data.array("/mavros/rc/override:channels")
    if depth.size or dvl_v.size or rc.size:
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        if depth.size:
            axes[0].plot(depth_t, depth.reshape(-1), lw=1.0)
        axes[0].set_ylabel("depth m")
        axes[0].grid(True, alpha=0.3)
        if dvl_v.size:
            axes[1].plot(dvl_t, dvl_v[:, 0], label="vx", lw=0.9)
            axes[1].plot(dvl_t, dvl_v[:, 1], label="vy", lw=0.9)
            axes[1].plot(dvl_t, dvl_v[:, 2], label="vz", lw=0.9)
            axes[1].plot(dvl_t, np.linalg.norm(dvl_v, axis=1), label="speed", lw=1.1, alpha=0.8)
            axes[1].legend(loc="upper right", ncol=4, fontsize=8)
        axes[1].set_ylabel("DVL m/s")
        axes[1].grid(True, alpha=0.3)
        if rc.size:
            channels = min(rc.shape[1], 8)
            for idx in range(channels):
                axes[2].plot(rc_t, rc[:, idx], label=f"ch{idx + 1}", lw=0.8)
            axes[2].legend(loc="upper right", ncol=4, fontsize=8)
        axes[2].set_ylabel("RC pwm")
        axes[2].set_xlabel("bag time s")
        axes[2].grid(True, alpha=0.3)
        fig.suptitle(data.name)
        fig.tight_layout()
        path = out_dir / f"{data.name}_depth_dvl_rc.png"
        plt.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(str(path))

    odom_t, odom_xyz = data.array("/odometry/filtered:xyz_m", 3)
    local_t, local_xyz = data.array("/mavros/local_position/odom:xyz_m", 3)
    tf_t, tf_xyz = data.array("/tf:odom->base_link:xyz_m", 3)
    if odom_xyz.size or local_xyz.size or tf_xyz.size:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        if odom_xyz.size:
            axes[0].plot(odom_xyz[:, 0], odom_xyz[:, 1], label="/odometry/filtered", lw=1.0)
            axes[1].plot(odom_t, odom_xyz[:, 2], label="/odometry/filtered z", lw=1.0)
        if local_xyz.size:
            axes[0].plot(local_xyz[:, 0], local_xyz[:, 1], label="/mavros/local_position/odom", lw=1.0)
            axes[1].plot(local_t, local_xyz[:, 2], label="/mavros local z", lw=1.0)
        if tf_xyz.size:
            axes[0].plot(tf_xyz[:, 0], tf_xyz[:, 1], label="/tf odom->base_link", lw=1.0)
            axes[1].plot(tf_t, tf_xyz[:, 2], label="/tf z", lw=1.0)
        axes[0].set_aspect("equal", adjustable="box")
        axes[0].set_xlabel("x m")
        axes[0].set_ylabel("y m")
        axes[1].set_xlabel("bag time s")
        axes[1].set_ylabel("z m")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
        fig.suptitle(f"{data.name} trajectory estimates")
        fig.tight_layout()
        path = out_dir / f"{data.name}_trajectory.png"
        plt.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(str(path))

    imu_t, gyro = data.array("/mavros/imu/data:gyro_rad_s", 3)
    _, accel = data.array("/mavros/imu/data:accel_m_s2", 3)
    if gyro.size or accel.size:
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        if gyro.size:
            for idx, axis in enumerate("xyz"):
                axes[0].plot(imu_t, gyro[:, idx], label=axis, lw=0.8)
            axes[0].legend(loc="upper right", ncol=3, fontsize=8)
        axes[0].set_ylabel("gyro rad/s")
        axes[0].grid(True, alpha=0.3)
        if accel.size:
            for idx, axis in enumerate("xyz"):
                axes[1].plot(imu_t, accel[:, idx], label=axis, lw=0.8)
            axes[1].legend(loc="upper right", ncol=3, fontsize=8)
        axes[1].set_ylabel("accel m/s^2")
        axes[1].set_xlabel("bag time s")
        axes[1].grid(True, alpha=0.3)
        fig.suptitle(f"{data.name} MAVROS IMU")
        fig.tight_layout()
        path = out_dir / f"{data.name}_imu.png"
        plt.savefig(path, dpi=150)
        plt.close(fig)
        paths.append(str(path))

    return paths


def fmt_float(value: Any, digits: int = 4) -> str:
    if value is None:
        return "n/a"
    try:
        value = float(value)
    except Exception:
        return str(value)
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}g}"


def signal_rate(summary: dict[str, Any], key: str) -> str:
    stats = summary.get("signals", {}).get(key, {})
    return fmt_float(stats.get("x", stats).get("median_rate_hz") if isinstance(stats.get("x"), dict) else stats.get("median_rate_hz"), 4)


def make_report(summaries: list[dict[str, Any]], plot_paths: list[str], out_dir: Path) -> str:
    lines: list[str] = []
    lines.append("# 2026-04-01 Real Robot ROS Bag Analysis")
    lines.append("")
    lines.append("## Bag inventory")
    lines.append("")
    lines.append("| bag | duration s | messages | topics | IMU Hz | DVL Hz | depth Hz | odom filtered Hz | local odom Hz |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for s in summaries:
        lines.append(
            "| {name} | {dur} | {msgs} | {topics} | {imu} | {dvl} | {depth} | {of} | {local} |".format(
                name=s["name"],
                dur=fmt_float(s["duration_s"], 5),
                msgs=int(s["message_count"]),
                topics=int(s["topic_count"]),
                imu=signal_rate(s, "/mavros/imu/data:gyro_rad_s"),
                dvl=signal_rate(s, "/dvl/twist:linear_m_s"),
                depth=signal_rate(s, "/depth/pose:depth_positive_m"),
                of=signal_rate(s, "/odometry/filtered:xyz_m"),
                local=signal_rate(s, "/mavros/local_position/odom:xyz_m"),
            )
        )
    lines.append("")

    lines.append("## Interface observations")
    lines.append("")
    for s in summaries:
        frames = s.get("frames", {})
        imu_frame = frames.get("/mavros/imu/data", {})
        dvl_frame = frames.get("/dvl/twist", {})
        depth_frame = frames.get("/depth/pose", {})
        lines.append(f"- `{s['name']}` frames: IMU `{imu_frame}`, DVL `{dvl_frame}`, depth `{depth_frame}`.")
        modes = s.get("state_modes", {})
        if modes:
            lines.append(f"- `{s['name']}` MAVROS modes: `{modes}`.")
        tf_top = s.get("tf_edges_top", [])[:8]
        if tf_top:
            lines.append(f"- `{s['name']}` top TF edges: `{tf_top}`.")
    lines.append("")

    lines.append("## Sensor statistics")
    lines.append("")
    lines.append("| bag | depth range m | DVL speed p95 m/s | DVL max gap s | IMU gyro HP std xyz rad/s | IMU accel HP std xyz m/s^2 |")
    lines.append("|---|---:|---:|---:|---|---|")
    for s in summaries:
        depth = s.get("derived", {}).get("depth_positive_m", {})
        dvl = s.get("derived", {}).get("dvl_speed_m_s", {})
        dvl_sig = s.get("signals", {}).get("/dvl/twist:linear_m_s", {}).get("x", {})
        gyro = s.get("derived", {}).get("/mavros/imu/data:gyro_noise_estimate", {})
        accel = s.get("derived", {}).get("/mavros/imu/data:accel_noise_estimate", {})
        lines.append(
            "| {name} | {drange} | {dvlp95} | {gap} | [{gx}, {gy}, {gz}] | [{ax}, {ay}, {az}] |".format(
                name=s["name"],
                drange=fmt_float(depth.get("range"), 4),
                dvlp95=fmt_float(dvl.get("p95"), 4),
                gap=fmt_float(dvl_sig.get("max_dt_s"), 4),
                gx=fmt_float(gyro.get("x"), 4),
                gy=fmt_float(gyro.get("y"), 4),
                gz=fmt_float(gyro.get("z"), 4),
                ax=fmt_float(accel.get("x"), 4),
                ay=fmt_float(accel.get("y"), 4),
                az=fmt_float(accel.get("z"), 4),
            )
        )
    lines.append("")

    lines.append("## Cross-sensor checks")
    lines.append("")
    lines.append("| bag | static pressure slope Pa/m | pressure-depth corr | static pressure fit residual std Pa | depth vs -filtered z residual std m | local odom vs filtered xyz residual norm p95 m |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for s in summaries:
        static_fit = s.get("derived", {}).get("static_pressure_vs_depth", {})
        depth_fit = s.get("derived", {}).get("depth_vs_negative_odometry_filtered_z", {})
        local_fit = s.get("derived", {}).get("local_odom_vs_odometry_filtered_xyz_offset_removed", {})
        lines.append(
            "| {name} | {slope} | {corr} | {pres_resid} | {depth_resid} | {local_resid} |".format(
                name=s["name"],
                slope=fmt_float(static_fit.get("estimated_rho_g_pa_per_m"), 5),
                corr=fmt_float(static_fit.get("correlation"), 4),
                pres_resid=fmt_float(static_fit.get("fit_residual_pa", {}).get("std"), 4),
                depth_resid=fmt_float(depth_fit.get("residual", {}).get("std"), 4),
                local_resid=fmt_float(local_fit.get("residual", {}).get("norm", {}).get("p95"), 4),
            )
        )
    lines.append("")

    lines.append("## Covariance values observed")
    lines.append("")
    for s in summaries:
        lines.append(f"### {s['name']}")
        for key in (
            "/mavros/imu/data:orientation_cov",
            "/mavros/imu/data:gyro_cov",
            "/mavros/imu/data:accel_cov",
            "/dvl/twist:linear_cov",
            "/depth/pose:position_cov",
            "/odometry/filtered:pose_position_cov",
            "/odometry/filtered:twist_linear_cov",
        ):
            cov = s.get("covariances", {}).get(key, {})
            if cov.get("count", 0):
                lines.append(f"- `{key}` mean diag: `{cov.get('mean_diag')}`, unique count: `{cov.get('unique_diag_count')}`")
        lines.append("")

    lines.append("## Control / command observations")
    lines.append("")
    for s in summaries:
        rc = s.get("derived", {}).get("rc_override_channels")
        joy = s.get("derived", {}).get("joy_axes")
        if rc:
            lines.append(f"- `{s['name']}` RC override median: `{rc['median'][:8]}`, p05: `{rc['p05'][:8]}`, p95: `{rc['p95'][:8]}`, active fraction: `{fmt_float(rc['active_fraction_gt_30us_from_median'], 4)}`.")
        if joy:
            lines.append(f"- `{s['name']}` joy axes p95 abs: `{joy['p95_abs']}`.")
    lines.append("")

    lines.append("## Simulator implications")
    lines.append("")
    lines.append("- The April 1 data confirms the bridge should publish `/mavros/imu/data` at about 20 Hz with `frame_id=fcu_link`, `/depth/pose` at 2 Hz with `frame_id=odom`, and `/dvl/twist` in `frame_id=dvl` when DVL bottom lock is available.")
    lines.append("- Actual water pressure variation is on `/mavros/imu/static_pressure` at about 2 Hz. `/mavros/imu/atm_pressure` is present at about 10 Hz but is constant near `0.24` in these bags, so it should not be treated as the depth pressure stream.")
    lines.append("- DVL is not guaranteed to be present: the 20:06 bag has zero `/dvl/twist` messages while `/dvl/position` exists. The simulator should support DVL dropout/bottom-lock failure, not only Gaussian noise.")
    lines.append("- `/mavros/local_position/*` is much slower than raw sensors, around 2.6-2.8 Hz in the runs where it exists. For EKF/controller testing, raw sensor rates and estimator output rates should remain separate.")
    lines.append("- The 20:20 bag contains RealSense depth/IR/color and camera IMU streams. For SLAM tests, the sim will eventually need camera topic compatibility or a camera bridge path, but the control-critical AUV dynamics still depend first on IMU, DVL, depth, RC/joy, and odometry.")
    lines.append("- Covariance values are mostly fixed constants. The measured high-pass residuals should be treated as operational noise-plus-motion, so use them as upper bounds unless you isolate a stationary segment.")
    lines.append("")

    if plot_paths:
        lines.append("## Generated plots")
        lines.append("")
        for path in plot_paths:
            rel = Path(path)
            try:
                rel = rel.relative_to(out_dir.parent.parent.parent)
            except Exception:
                pass
            lines.append(f"- `{path}`")
        lines.append("")

    report = "\n".join(lines)
    (out_dir / "report.md").write_text(report, encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze April 1 real robot ROS bags without reading image payload topics.")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    bag_paths = discover_bags(args.root)
    if not bag_paths:
        raise SystemExit(f"No .db3 bags found under {args.root}")

    summaries: list[dict[str, Any]] = []
    plot_paths: list[str] = []
    for db_path in bag_paths:
        print(f"[analyze] {db_path}", flush=True)
        data = read_bag(db_path)
        summary = summarize_bag(data)
        summaries.append(summary)
        bag_out = args.out / data.name
        bag_out.mkdir(parents=True, exist_ok=True)
        (bag_out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        if not args.no_plots:
            plot_paths.extend(plot_bag(data, bag_out))

    all_summary = {"bags": summaries, "plots": plot_paths}
    (args.out / "summary.json").write_text(json.dumps(all_summary, indent=2, ensure_ascii=False), encoding="utf-8")
    make_report(summaries, plot_paths, args.out)
    print(f"[analyze] wrote {args.out / 'report.md'}", flush=True)


if __name__ == "__main__":
    main()
