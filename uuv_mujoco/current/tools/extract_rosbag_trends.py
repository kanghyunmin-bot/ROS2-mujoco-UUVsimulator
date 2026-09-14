"""Read ROS 2 SQLite sensor/path data offline (optional rosbags dependency)."""

from __future__ import annotations

import argparse
import hashlib
import json
import sqlite3
from pathlib import Path

import numpy as np


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def extract(database, output, source_zip):
    """Extract paired values/times; decoding errors stop without shifted pairing."""
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    if output.exists() and any(output.iterdir()):
        raise ValueError("Refusing to overwrite extracted data")
    store = get_typestore(Stores.ROS2_HUMBLE)
    definitions = {}
    root = Path(__file__).resolve().parents[3]
    for path in (root / "rospkg/src/dvl_msgs/msg").glob("*.msg"):
        definitions.update(
            get_types_from_msg(path.read_text(), "dvl_msgs/msg/" + path.stem)
        )
    for name, definition in {
        "RCOut": "std_msgs/Header header\nuint16[] channels",
        "OverrideRCIn": "uint16[18] channels",
        "State": "std_msgs/Header header\nbool connected\nbool armed\nbool guided\nbool manual_input\nstring mode\nuint8 system_status",
    }.items():
        definitions.update(get_types_from_msg(definition, "mavros_msgs/msg/" + name))
    store.register(definitions)
    connection = sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True)
    if connection.execute("PRAGMA quick_check").fetchone()[0] != "ok":
        raise ValueError("SQLite integrity check failed")
    origin_ns = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    if origin_ns is None:
        raise ValueError("Empty bag")
    origin = origin_ns / 1e9
    wanted = {
        "/mavros/imu/data",
        "/mavros/imu/data_raw",
        "/mavros/imu/static_pressure",
        "/dvl/data",
        "/depth/pose",
        "/mavros/rc/out",
        "/mavros/rc/override",
        "/mavros/state",
        "/mavros/local_position/odom",
        "/odometry/filtered",
        "/battery",
        "/mavros/battery",
    }
    numeric, odometry, profile, row_counts = {}, {}, {}, {}
    topics = connection.execute("SELECT id,name,type FROM topics").fetchall()
    for topic_id, name, typ in topics:
        count, start, end = connection.execute(
            "SELECT COUNT(*),MIN(timestamp),MAX(timestamp) FROM messages WHERE topic_id=?",
            (topic_id,),
        ).fetchone()
        if not count:
            continue
        profile[name] = {
            "type": typ,
            "count": count,
            "start": start / 1e9,
            "end": end / 1e9,
        }
        if name not in wanted:
            continue
        values, times, positions, states, nanoseconds = [], [], [], [], []
        for timestamp, blob in connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
            (topic_id,),
        ):
            # Never skip a malformed packet then pair subsequent values with
            # a shorter prefix of the source timestamp array.
            message = store.deserialize_cdr(blob, typ)
            stamp_ns = (
                (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
                if hasattr(message, "header")
                else timestamp
            )
            record, header = timestamp / 1e9, stamp_ns / 1e9
            nanoseconds.append([timestamp, stamp_ns])
            times.append([record, header])
            if typ == "sensor_msgs/msg/Imu":
                q, w, a = (
                    message.orientation,
                    message.angular_velocity,
                    message.linear_acceleration,
                )
                value = [q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z]
            elif name == "/dvl/data":
                v = message.velocity
                value = [
                    v.x,
                    v.y,
                    v.z,
                    message.altitude,
                    message.velocity_valid,
                    message.fom,
                    message.status,
                    message.time_of_validity,
                    message.time_of_transmission,
                ]
            elif name == "/depth/pose":
                value = [message.pose.pose.position.z]
            elif typ == "sensor_msgs/msg/FluidPressure":
                value = [message.fluid_pressure]
            elif typ == "sensor_msgs/msg/BatteryState":
                # Preserve topic identity: MAVROS and a battery bridge may
                # publish incompatible sources on /mavros/battery. Pack
                # telemetry is not automatically a measured ESC-bus voltage.
                value = [message.voltage, message.current, message.percentage]
                profile[name]["columns"] = [
                    "receipt_epoch_s",
                    "voltage_v",
                    "current_a",
                    "percentage",
                ]
                profile[name]["voltage_reference"] = "unverified_pack_or_fcu_source"
            elif hasattr(message, "channels"):
                value = list(message.channels)
            elif name == "/mavros/state":
                states.append([record, message.connected, message.armed, message.mode])
                value = None
            elif typ == "nav_msgs/msg/Odometry":
                p, q, v = (
                    message.pose.pose.position,
                    message.pose.pose.orientation,
                    message.twist.twist.linear,
                )
                positions.append(
                    [
                        record - origin,
                        header - origin,
                        p.x,
                        p.y,
                        p.z,
                        q.x,
                        q.y,
                        q.z,
                        q.w,
                        v.x,
                        v.y,
                        v.z,
                    ]
                )
                value = None
            else:
                raise ValueError("Unsupported selected message: " + typ)
            if value is not None:
                values.append([record, *value])
        key = name.replace("/", "__")
        numeric[key + "__times"] = np.asarray(times)
        numeric[key + "__times_ns"] = np.asarray(nanoseconds, dtype=np.int64)
        if values:
            numeric[key] = np.asarray(values, dtype=float)
        if positions:
            odometry[key] = np.asarray(positions, dtype=float)
        if states:
            profile[name]["transitions"] = [
                s for i, s in enumerate(states) if i == 0 or s[1:] != states[i - 1][1:]
            ]
        row_counts[name] = len(times)
    connection.close()
    required_numeric = {
        "__mavros__imu__data",
        "__dvl__data",
        "__depth__pose",
        "__mavros__imu__static_pressure",
    }
    required_odom = {"__mavros__local_position__odom", "__odometry__filtered"}
    if not required_numeric <= numeric.keys() or not required_odom <= odometry.keys():
        raise ValueError("Missing required sensor/path streams")
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / "numeric.npz", **numeric)
    np.savez_compressed(output / "odometry.npz", **odometry)
    (output / "profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    manifest = {
        "source": str(source_zip),
        "sha256": sha256(source_zip),
        "sqlite_path": str(database),
        "sqlite_sha256": sha256(database),
        "sqlite_quick_check": "ok",
        "record_origin_ns": origin_ns,
        "decode_errors": 0,
        "decoded_topic_counts": row_counts,
        "derived_sha256": {
            p.name: sha256(p)
            for p in (
                output / "numeric.npz",
                output / "odometry.npz",
                output / "profile.json",
            )
        },
    }
    (output / "source.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        json.dumps(
            {"decoded_topic_counts": row_counts, "output": str(output)}, indent=2
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--database",
        type=Path,
        required=True,
        help="One extracted ROS 2 .db3; never opens ROS transport",
    )
    parser.add_argument(
        "--source_zip",
        type=Path,
        required=True,
        help="Original archive for provenance hashing",
    )
    parser.add_argument("--output_dir", type=Path, required=True)
    args = parser.parse_args()
    extract(args.database, args.output_dir, args.source_zip)
