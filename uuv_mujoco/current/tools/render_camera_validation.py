#!/usr/bin/env python3
"""Render actual front/hand RGB comparisons and bounded local cost measurements.

Run with MUJOCO_GL=egl for headless GPU rendering. Measurements are static scene
microbenchmarks; they do not establish live SITL real-time factor or collection
readiness. No simulation, GUI, or stored user settings are changed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from bridge.ros2_stereo_image import (
    CAMERA_NAMES,
    close_stereo_image_renderers,
    configure_stereo_image_runtime,
    render_camera_frame,
    render_camera_rgb,
)
from bridge.underwater_camera_sensor_model import (
    UnderwaterCameraSensorModel,
    load_underwater_camera_profile,
)


def _stats(samples: list[float]) -> dict[str, float]:
    return {
        "p50_ms": float(np.median(samples)),
        "p95_ms": float(np.percentile(samples, 95)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=CURRENT / "scenes/research_pool_slam_scene.xml")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--quality", choices=("low", "scene"), default="low")
    args = parser.parse_args()
    if args.frames < 1 or args.warmup < 0:
        parser.error("frames must be positive and warmup non-negative")
    scene = args.scene.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    profile_path = CURRENT / "config/sensor_models/imx219_underwater_pool_lite.json"
    model = mujoco.MjModel.from_xml_path(str(scene))
    if args.quality == "low":
        model.vis.quality.shadowsize = 1024
        model.vis.quality.offsamples = 1
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    owner = SimpleNamespace(model=model, _water_surface_z=0.0)
    with patch.dict(os.environ, {
        "ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE": "1",
        "ROS2_UUV_CAMERA_SENSOR_MODEL_CONFIG": str(profile_path),
    }):
        configure_stereo_image_runtime(
            owner, publish_images=True, image_width=args.width,
            image_height=args.height, image_hz=15,
        )
    before_state = (data.qpos.copy(), data.qvel.copy(), float(data.time), model.geom_rgba.copy())
    report = {
        "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "profile": str(profile_path),
        "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
        "mujoco_version": mujoco.__version__,
        "backend": os.environ.get("MUJOCO_GL", "default"),
        "width": owner._stereo_image_width,
        "height": owner._stereo_image_height,
        "samples": args.frames,
        "warmup": args.warmup,
        "shadowsize": int(model.vis.quality.shadowsize),
        "offsamples": int(model.vis.quality.offsamples),
        "scope": "static warm-render microbenchmark; no live SITL/GUI or end-to-end collection timing",
        "cameras": {},
    }
    rows = []
    try:
        for camera in CAMERA_NAMES:
            sensor = owner._camera_sensor_runtimes[camera].model
            prior = UnderwaterCameraSensorModel(
                load_underwater_camera_profile(), calibration=sensor.calibration,
                camera_name=camera,
            )
            raw_cost, capture_cost, lite_cost, prior_cost, total_cost = [], [], [], [], []
            for sequence in range(args.warmup + args.frames):
                start = time.perf_counter()
                render_camera_rgb(owner, camera, data)
                raw_end = time.perf_counter()
                frame = render_camera_frame(owner, camera, data)
                captured = time.perf_counter()
                rgb, diagnostics = sensor.process(
                    frame.rgb, sequence=sequence,
                    optical_path_length_m=frame.optical_path_length_m,
                )
                processed = time.perf_counter()
                prior.process(frame.rgb, sequence=sequence)
                prior_end = time.perf_counter()
                if sequence >= args.warmup:
                    raw_cost.append((raw_end - start) * 1000)
                    capture_cost.append((captured - raw_end) * 1000)
                    lite_cost.append((processed - captured) * 1000)
                    total_cost.append((processed - raw_end) * 1000)
                    prior_cost.append((prior_end - processed) * 1000)
            repeated, _ = sensor.process(
                frame.rgb, sequence=sequence,
                optical_path_length_m=frame.optical_path_length_m,
            )
            if not np.array_equal(repeated, rgb):
                raise RuntimeError("camera image model is not deterministic")
            for suffix, pixels in (("ideal", frame.rgb), ("pool_lite", rgb)):
                path = output / f"{camera}_{suffix}.png"
                if not cv2.imwrite(str(path), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError(f"failed to write {path}")
            row = np.concatenate((frame.rgb, rgb), axis=1)
            row = cv2.cvtColor(row, cv2.COLOR_RGB2BGR)
            banner = np.zeros((28, row.shape[1], 3), dtype=np.uint8)
            cv2.putText(banner, f"{camera}: ideal RGB", (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
            cv2.putText(banner, "Pool lite: depth-aware water optics", (rgb.shape[1] + 8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
            rows.append(np.concatenate((banner, row), axis=0))
            paths = frame.optical_path_length_m
            report["cameras"][camera] = {
                "rgb_only": _stats(raw_cost),
                "rgb_depth_and_projection": _stats(capture_cost),
                "pool_lite_postprocess": _stats(lite_cost),
                "original_prior_postprocess": _stats(prior_cost),
                "pool_lite_capture_and_process": _stats(total_cost),
                "optical_path_min_m": float(paths.min()),
                "optical_path_max_m": float(paths.max()),
                "optical_path_mean_m": diagnostics.optical_path_length_m,
                "intrinsics_k": list(sensor.calibration.k),
                "same_sequence_deterministic": True,
            }
            prior.close()
        cv2.imwrite(str(output / "camera_comparison.png"), np.concatenate(rows, axis=0))
        unchanged = (
            np.array_equal(before_state[0], data.qpos)
            and np.array_equal(before_state[1], data.qvel)
            and before_state[2] == data.time
            and np.array_equal(before_state[3], model.geom_rgba)
        )
        if not unchanged:
            raise RuntimeError("camera preview mutated simulation state")
        report["simulation_state_unchanged"] = True
        report["two_camera_15hz_serial_budget_percent"] = 1.5 * sum(
            item["pool_lite_capture_and_process"]["p50_ms"]
            for item in report["cameras"].values()
        )
        (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        close_stereo_image_renderers(owner)


if __name__ == "__main__":
    main()
