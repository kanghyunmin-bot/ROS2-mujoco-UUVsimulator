#!/usr/bin/env python3
"""Recover native navigation inputs and sensor telemetry without resampling.

The source manifest is the existing ``*_source.json`` bag inventory. Original
bag receipt times, ROS header times, MAVLink payload times and packet bytes
are retained separately. This does not reconstruct missing FCU sensor history.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import numpy as np
from pymavlink.dialects.v20 import ardupilotmega
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


MAVLINK_DEFINITION = """std_msgs/Header header
uint8 framing_status
uint8 magic
uint8 len
uint8 incompat_flags
uint8 compat_flags
uint8 seq
uint8 sysid
uint8 compid
uint32 msgid
uint16 checksum
uint64[<=33] payload64
uint8[<=13] signature
"""

SENSOR_TOPICS = {
    "/mavros/imu/data": "imu",
    "/mavros/imu/data_raw": "imu_raw",
    "/mavros/imu/mag": "mag",
    "/mavros/imu/static_pressure": "pressure",
}
VPD_COLUMNS = [
    "receipt_s", "header_s", "time_usec", "time_delta_usec",
    "angle_delta_x", "angle_delta_y", "angle_delta_z",
    "position_delta_x", "position_delta_y", "position_delta_z",
    "confidence", "sysid", "compid", "seq",
]


def mavlink_frame(message) -> bytes:
    """Rebuild a recorded MAVLink frame for checksum validation."""
    payload = np.asarray(message.payload64, dtype="<u8").tobytes()[:message.len]
    if message.magic == 253:
        header = bytes([
            message.magic, message.len, message.incompat_flags,
            message.compat_flags, message.seq, message.sysid, message.compid,
        ]) + int(message.msgid).to_bytes(3, "little")
    elif message.magic == 254:
        header = bytes([
            message.magic, message.len, message.seq, message.sysid,
            message.compid, message.msgid,
        ])
    else:
        raise ValueError(f"Unsupported MAVLink magic {message.magic}")
    result = header + payload + int(message.checksum).to_bytes(2, "little")
    if message.magic == 253 and message.incompat_flags & 1:
        result += bytes(message.signature)
    return result


def header_relative_s(message, start_ns: int) -> float:
    stamp = message.header.stamp
    return (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) - start_ns) / 1e9


def vector(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def sensor_values(message, kind: str) -> list[float]:
    if kind in {"imu", "imu_raw"}:
        q = message.orientation
        return [q.x, q.y, q.z, q.w, *vector(message.angular_velocity),
                *vector(message.linear_acceleration)]
    if kind == "mag":
        return vector(message.magnetic_field)
    return [float(message.fluid_pressure)]


def extract(source_manifest: Path, output_dir: Path, clock: str) -> dict:
    """Export native events; refuse to overwrite an existing experiment."""
    source = json.loads(source_manifest.read_text())
    database = Path(source["source"]).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(get_types_from_msg(MAVLINK_DEFINITION, "mavros_msgs/msg/Mavlink"))
    parser = ardupilotmega.MAVLink(None)
    arrays = {kind: [] for kind in SENSOR_TOPICS.values()}
    arrays.update(raw_imu_mavlink=[], ahrs_mavlink=[])
    vpd, frames, fcu_frames, failures = [], [], [], []
    counts, source_counts = Counter(), Counter()
    start_ns = int(source["start_ns"])
    with sqlite3.connect(database.as_uri() + "?mode=ro&immutable=1", uri=True) as db:
        topics = db.execute("SELECT id,name,type FROM topics ORDER BY id").fetchall()
        for topic_id, topic, type_name in topics:
            if topic not in SENSOR_TOPICS and topic not in {"/uas1/mavlink_sink", "/uas1/mavlink_source"}:
                continue
            query = "SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id"
            for row_id, timestamp, data in db.execute(query, (topic_id,)):
                message = store.deserialize_cdr(data, type_name)
                receipt_s = (timestamp - start_ns) / 1e9
                header_s = header_relative_s(message, start_ns)
                if topic in SENSOR_TOPICS:
                    kind = SENSOR_TOPICS[topic]
                    arrays[kind].append([receipt_s, header_s, *sensor_values(message, kind)])
                    continue
                msgid = int(message.msgid)
                sink = topic == "/uas1/mavlink_sink"
                (counts if sink else source_counts)[msgid] += 1
                if (sink and msgid != 11011) or (not sink and msgid not in {27, 163}):
                    continue
                try:
                    if message.framing_status != 1:
                        raise ValueError(f"framing_status={message.framing_status}")
                    frame = mavlink_frame(message)
                    decoded = parser.decode(bytearray(frame))
                except Exception as error:
                    failures.append({"row_id": row_id, "receipt_s": receipt_s, "msgid": msgid,
                                     "topic": topic, "error": str(error)})
                    continue
                identity = [int(message.sysid), int(message.compid), int(message.seq)]
                frame_row = {"row_id": row_id, "receipt_s": receipt_s,
                             "header_s": header_s, "frame_hex": frame.hex()}
                if msgid == 11011:
                    vpd.append([
                        receipt_s, header_s, int(decoded.time_usec), int(decoded.time_delta_usec),
                        *decoded.angle_delta, *decoded.position_delta, float(decoded.confidence), *identity,
                    ])
                    frames.append(frame_row)
                else:
                    fcu_frames.append({"msgid": msgid, **frame_row})
                    if msgid == 27:
                        arrays["raw_imu_mavlink"].append([
                            receipt_s, header_s, int(decoded.time_usec),
                            *[int(getattr(decoded, axis + suffix))
                              for suffix in ("acc", "gyro", "mag") for axis in "xyz"], *identity,
                        ])
                    else:
                        arrays["ahrs_mavlink"].append([
                            receipt_s, header_s, decoded.omegaIx, decoded.omegaIy, decoded.omegaIz,
                            decoded.accel_weight, decoded.renorm_val, decoded.error_rp, decoded.error_yaw,
                            *identity,
                        ])
    arrays["vpd"] = vpd
    np.savez_compressed(output_dir / "native_events.npz", **{
        key: np.asarray(values, dtype=np.float64) for key, values in arrays.items()
    })
    with (output_dir / "vpd.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_s", *VPD_COLUMNS])
        for row in vpd:
            writer.writerow([row[0 if clock == "receipt" else 1], *row])
    with (output_dir / "vpd_frames.jsonl").open("w") as stream:
        for row in frames:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    with (output_dir / "fcu_sensor_frames.jsonl").open("w") as stream:
        for row in fcu_frames:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    summaries = {}
    for key, rows in arrays.items():
        if not rows:
            summaries[key] = {"samples": 0}
            continue
        data = np.asarray(rows)
        delta = np.diff(data[:, 0])
        summaries[key] = {
            "samples": len(data), "first_receipt_s": float(data[0, 0]),
            "last_receipt_s": float(data[-1, 0]),
            "median_interval_s": float(np.median(delta)) if len(delta) else None,
            "max_interval_s": float(np.max(delta)) if len(delta) else None,
            "receipt_nonmonotonic": int(np.sum(delta < 0)),
            "header_nonmonotonic": int(np.sum(np.diff(data[:, 1]) < 0)),
        }
    receipt = {
        "source_manifest": str(source_manifest.resolve()),
        "source_manifest_sha256": hashlib.sha256(source_manifest.read_bytes()).hexdigest(),
        "database": str(database), "database_size": database.stat().st_size,
        "bag_start_ns": start_ns, "csv_clock": clock,
        "sink_message_id_counts": dict(counts), "source_message_id_counts": dict(source_counts),
        "streams": summaries, "vpd_columns": VPD_COLUMNS,
        "vpd_crc_failures": [f for f in failures if f["msgid"] == 11011],
        "mavlink_crc_failures": failures,
        "sensor_columns": {
            "imu": ["receipt_s", "header_s", "qx", "qy", "qz", "qw",
                    "wx_rad_s", "wy_rad_s", "wz_rad_s", "ax_m_s2", "ay_m_s2", "az_m_s2"],
            "imu_raw": "Same columns as imu; orientation can be invalid/absent",
            "mag": ["receipt_s", "header_s", "bx_T", "by_T", "bz_T"],
            "pressure": ["receipt_s", "header_s", "absolute_pressure_Pa"],
            "raw_imu_mavlink": ["receipt_s", "header_s", "time_usec",
                                "xacc_mg", "yacc_mg", "zacc_mg",
                                "xgyro_mrad_s", "ygyro_mrad_s", "zgyro_mrad_s",
                                "xmag_milligauss", "ymag_milligauss", "zmag_milligauss",
                                "sysid", "compid", "seq"],
            "ahrs_mavlink": ["receipt_s", "header_s", "omegaIx_rad_s", "omegaIy_rad_s", "omegaIz_rad_s",
                             "accel_weight", "renorm_val", "error_rp", "error_yaw", "sysid", "compid", "seq"],
        },
        "scope": (
            "Exact recorded VISION_POSITION_DELTA, RAW_IMU and AHRS payloads with checksum validation. "
            "No smoothing, correction, fitted lag, gap filling, or packet deduplication. "
            "ROS telemetry is not a complete raw FCU sensor history. Native VPD is "
            "the field input, including its defects; it is not physical ground truth."
        ),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    (output_dir / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source_manifest", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--clock", choices=["receipt", "header"], default="receipt")
    args = parser.parse_args()
    receipt = extract(args.source_manifest, args.output_dir, args.clock)
    print(json.dumps({"output_dir": str(args.output_dir), "streams": receipt["streams"],
                      "vpd_crc_failures": len(receipt["vpd_crc_failures"])}))
    return int(bool(receipt["mavlink_crc_failures"]))


if __name__ == "__main__":
    raise SystemExit(main())
