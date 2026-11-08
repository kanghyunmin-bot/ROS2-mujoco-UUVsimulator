"""Read declared DVL schemas and frame names across field-recording dates."""

from pathlib import Path
import sqlite3

import numpy as np
import pytest

pytest.importorskip("rosbags")
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from audit_rosbag_frame_contract import audit

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("package,frame", [
    ("dvl_msgs", "dvl"),
    ("auv_dvl_a50_msg", "dvl"),
    ("auv_dvl_a50_msg", "dvl_link"),
])
def test_bag_declared_dvl_type_and_frame_are_used(tmp_path, package, frame):
    store = get_typestore(Stores.ROS2_HUMBLE)
    definitions = {}
    for name in ["DVL", "DVLBeam"]:
        definitions.update(get_types_from_msg(
            (ROOT / "rospkg/src" / package / "msg" / (name + ".msg")).read_text(),
            f"{package}/msg/{name}",
        ))
    store.register(definitions)
    types = store.types
    stamp = types["builtin_interfaces/msg/Time"](1, 0)
    header = types["std_msgs/msg/Header"](stamp, frame)
    vector = types["geometry_msgs/msg/Vector3"]
    dvl_type = f"{package}/msg/DVL"
    message = types[dvl_type](
        header, 100.0, vector(0.2, -0.1, 0.0), 0.001, np.zeros(9),
        1.0, [], True, 0, 1000000, 1001000, "json_v3.3",
    )
    transform = types["geometry_msgs/msg/TransformStamped"](
        types["std_msgs/msg/Header"](stamp, "base_link"), frame,
        types["geometry_msgs/msg/Transform"](
            vector(-0.03, 0.02, -0.10),
            types["geometry_msgs/msg/Quaternion"](0.0, 0.0, 0.0, 1.0),
        ),
    )
    database = tmp_path / "field.db3"
    with sqlite3.connect(database) as db:
        db.executescript(
            "CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);"
            "CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);"
            "INSERT INTO topics VALUES(1,'/tf_static','tf2_msgs/msg/TFMessage');"
        )
        db.execute("INSERT INTO topics VALUES(2,'/dvl/data',?)", (dvl_type,))
        db.executemany("INSERT INTO messages VALUES(?,?,?,?)", [
            (1, 1, 100, bytes(store.serialize_cdr(types["tf2_msgs/msg/TFMessage"]([transform]), "tf2_msgs/msg/TFMessage"))),
            (2, 2, 200, bytes(store.serialize_cdr(message, dvl_type))),
        ])
    before = database.read_bytes()
    result = audit(database)
    assert result["message_frames"]["/dvl/data"] == {frame: 1}
    observed = result["sensor_transforms"][frame]
    assert observed["status"] == "recorded_static_tf"
    np.testing.assert_allclose(np.asarray(observed["base_from_sensor"])[:3, 3], [-0.03, 0.02, -0.10])
    assert database.read_bytes() == before
    if frame != "dvl":
        assert result["sensor_transforms"]["dvl"]["status"] == "missing"
