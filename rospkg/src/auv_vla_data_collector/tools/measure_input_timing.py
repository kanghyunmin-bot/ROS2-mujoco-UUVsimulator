"""Read-only camera/sensor freshness measurement at the collector's ROS cadence."""

import argparse
import json
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from kmu26_auv_vla_data_collector.collector import VlaDataCollector

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--duration", type=float, default=25)
p.add_argument("--use_sim_time", action="store_true")
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
if a.duration <= 0:
    p.error("duration must be positive")
with tempfile.TemporaryDirectory(prefix="vla-input-probe-") as temporary:
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            "__node:=vla_input_probe",
            "-p",
            f"use_sim_time:={str(a.use_sim_time).lower()}",
            "-p",
            f"dataset_root:={temporary}",
        ]
    )
    node = VlaDataCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    rows = []
    start, last = time.monotonic(), -float("inf")
    try:
        while time.monotonic() - start < a.duration:
            executor.spin_once(timeout_sec=0.01)
            now = node._now()
            if now - last < 0.1:
                continue
            last = now
            row = {"ros_time": now, "wall_time": time.monotonic()}
            for key in ("ego", "release", "imu", "depth", "dvl_twist", "dvl_data"):
                v = getattr(node, "_" + key)
                row[key] = (
                    None
                    if v is None
                    else {
                        "source_time": v.source_time,
                        "receipt_time": v.received_time,
                        "source_age": now - v.source_time,
                        "frame": v.frame_id,
                        "fresh": node._is_fresh(v, now, node._max_sensor_age),
                    }
                )
            rows.append(row)
        summary = {}
        for key in ("ego", "release", "imu", "depth", "dvl_twist", "dvl_data"):
            values = [row[key] for row in rows if row[key] is not None]
            summary[key] = {
                "received_samples": len(values),
                "total_samples": len(rows),
                "fresh_ratio": sum(v["fresh"] for v in values) / max(1, len(rows)),
                "unique_ratio": len({v["source_time"] for v in values})
                / max(1, len(rows)),
            }
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(
            json.dumps({"summary": summary, "rows": rows}, indent=2) + "\n"
        )
        print(json.dumps(summary, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
