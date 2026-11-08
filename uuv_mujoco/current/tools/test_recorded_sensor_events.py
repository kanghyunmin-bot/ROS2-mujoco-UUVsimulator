"""Validate lossless native-event recovery, corruption handling and clocks."""

import importlib.util
import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("pymavlink")
pytest.importorskip("rosbags")

SCRIPT = Path(__file__).resolve().parents[1] / "debug/controller_parity_412/extract_recorded_sensor_events.py"
SPEC = importlib.util.spec_from_file_location("recorded_events", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def recorded_message(store, frame: bytes):
    types = store.types
    return types["mavros_msgs/msg/Mavlink"](
        types["std_msgs/msg/Header"](types["builtin_interfaces/msg/Time"](1775133982, 123456789), ""),
        1, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6],
        int.from_bytes(frame[7:10], "little"),
        int.from_bytes(frame[10 + frame[1]:12 + frame[1]], "little"),
        np.frombuffer(frame[10:10 + frame[1]].ljust(8 * ((frame[1] + 7) // 8), b"\0"), dtype="<u8"),
        np.array([], dtype=np.uint8),
    )


def source(tmp_path: Path, corrupt: bool = False):
    store = MODULE.get_typestore(MODULE.Stores.ROS2_HUMBLE)
    store.register(MODULE.get_types_from_msg(MODULE.MAVLINK_DEFINITION, "mavros_msgs/msg/Mavlink"))
    mav = MODULE.ardupilotmega.MAVLink(None, srcSystem=255, srcComponent=240)
    packet = mav.vision_position_delta_encode(
        1775133982123456, 107303, [-0.35, 0.12, 0.02], [0.001, -0.002, 0.003], 99.9,
    ).pack(mav)
    message = recorded_message(store, packet)
    if corrupt:
        message.checksum ^= 1
    data = store.serialize_cdr(message, "mavros_msgs/msg/Mavlink").tobytes()
    db_path = tmp_path / "bag.db3"
    with sqlite3.connect(db_path) as db:
        db.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)")
        db.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
        db.execute("INSERT INTO topics VALUES(1,'/uas1/mavlink_sink','mavros_msgs/msg/Mavlink')")
        # Same packet recorded twice must remain twice. Replaying the original
        # event stream must not silently deduplicate by sequence or payload time.
        db.executemany("INSERT INTO messages VALUES(?,1,?,?)", [
            (1, 1775133982123456800, data), (2, 1775133982123456900, data),
        ])
    manifest = tmp_path / "source.json"
    manifest.write_text(json.dumps({"source": str(db_path), "start_ns": 1775133982000000000}))
    return manifest, db_path, packet


def test_native_packet_bytes_duplicates_and_clock_precision(tmp_path: Path) -> None:
    manifest, database, packet = source(tmp_path)
    original_database = database.read_bytes()
    output = tmp_path / "output"
    receipt = MODULE.extract(manifest, output, "receipt")
    rows = [json.loads(line) for line in (output / "vpd_frames.jsonl").read_text().splitlines()]
    assert [bytes.fromhex(row["frame_hex"]) for row in rows] == [packet, packet]
    assert rows[0]["header_s"] == pytest.approx(0.123456789, abs=1e-12)
    assert rows[0]["receipt_s"] == pytest.approx(0.123456800, abs=1e-12)
    assert receipt["streams"]["vpd"]["samples"] == 2
    assert receipt["vpd_crc_failures"] == []
    assert database.read_bytes() == original_database
    before = (output / "native_events.npz").read_bytes()
    with pytest.raises(FileExistsError):
        MODULE.extract(manifest, output, "receipt")
    assert (output / "native_events.npz").read_bytes() == before


def test_corrupted_packets_are_reported_and_excluded(tmp_path: Path) -> None:
    manifest, _, _ = source(tmp_path, corrupt=True)
    receipt = MODULE.extract(manifest, tmp_path / "output", "header")
    assert receipt["streams"]["vpd"]["samples"] == 0
    assert len(receipt["vpd_crc_failures"]) == 2


def test_header_subtraction_preserves_nanoseconds() -> None:
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1775133982, nanosec=123456789)))
    assert MODULE.header_relative_s(message, 1775133982000000000) == 0.123456789


def add_fcu_sensor_packets(database: Path, corrupt_raw: bool = False) -> list[bytes]:
    store = MODULE.get_typestore(MODULE.Stores.ROS2_HUMBLE)
    store.register(MODULE.get_types_from_msg(MODULE.MAVLINK_DEFINITION, "mavros_msgs/msg/Mavlink"))
    mav = MODULE.ardupilotmega.MAVLink(None, srcSystem=1, srcComponent=1)
    ahrs = mav.ahrs_encode(.01, -.02, .003, 0., 0., .04, .05).pack(mav)
    raw = mav.raw_imu_encode(4944606850, 43, 85, -903, -11, -27, 14, 120, -31, 400).pack(mav)
    packets = [ahrs, raw, ahrs]
    rows = []
    for index, packet in enumerate(packets):
        message = recorded_message(store, packet)
        if corrupt_raw and index == 1:
            message.checksum ^= 1
        blob = store.serialize_cdr(message, "mavros_msgs/msg/Mavlink").tobytes()
        rows.append((index + 3, 1775133982123457000 + index * 100, blob))
    with sqlite3.connect(database) as db:
        db.execute("INSERT INTO topics VALUES(2,'/uas1/mavlink_source','mavros_msgs/msg/Mavlink')")
        db.executemany("INSERT INTO messages VALUES(?,2,?,?)", rows)
    return packets


def test_fcu_raw_imu_and_ahrs_preserve_units_bytes_and_duplicates(tmp_path: Path) -> None:
    manifest, database, _ = source(tmp_path)
    packets = add_fcu_sensor_packets(database)
    original = database.read_bytes()
    output = tmp_path / "output"
    receipt = MODULE.extract(manifest, output, "receipt")
    arrays = np.load(output / "native_events.npz")
    np.testing.assert_array_equal(arrays["raw_imu_mavlink"][:, 2:12],
                                  [[4944606850, 43, 85, -903, -11, -27, 14, 120, -31, 400]])
    np.testing.assert_array_equal(arrays["ahrs_mavlink"][:, 2:5],
                                  np.array([[.01, -.02, .003]] * 2, dtype=np.float32).astype(float))
    frames = [json.loads(row) for row in (output / "fcu_sensor_frames.jsonl").read_text().splitlines()]
    assert [bytes.fromhex(row["frame_hex"]) for row in frames] == packets
    assert receipt["mavlink_crc_failures"] == []
    assert receipt["source_message_id_counts"] == {163: 2, 27: 1}
    assert database.read_bytes() == original


def test_fcu_corruption_does_not_mislabel_a_vpd_failure(tmp_path: Path) -> None:
    manifest, database, _ = source(tmp_path)
    add_fcu_sensor_packets(database, corrupt_raw=True)
    receipt = MODULE.extract(manifest, tmp_path / "output", "receipt")
    assert receipt["streams"]["raw_imu_mavlink"]["samples"] == 0
    assert receipt["streams"]["ahrs_mavlink"]["samples"] == 2
    assert receipt["vpd_crc_failures"] == []
    assert len(receipt["mavlink_crc_failures"]) == 1
    assert receipt["mavlink_crc_failures"][0]["msgid"] == 27
