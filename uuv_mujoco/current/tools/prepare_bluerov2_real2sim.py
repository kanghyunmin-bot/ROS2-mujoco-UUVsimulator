"""Package the BlueROV2 MJCF and normalized DFKI tank trials for Real2Sim."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys

import mujoco

from real2sim_auto import write_json

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))
from physics.thruster_mapping import (
    BLUEROV2_DFKI_SERVO_SIGNS,
    BLUEROV2_HORIZONTAL_THRUSTERS,
    BLUEROV2_PROP_SPIN_SIGNS,
    BLUEROV2_SERVO_MAP,
    BLUEROV2_T200_FORWARD_SIGNS,
)

TRAIN = ("random", "circle", "5m", "10m", "15m")
HOLDOUT = ("serpentine", "7m", "12m", "14m")


def source_sensors(scene: Path) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene))
    sensors = {}
    for name, site in (("imu", "imu_site"), ("pressure", "bar30_site")):
        sensor = model.site(site)
        sensors[name] = {"pos_m": sensor.pos.tolist(), "quat_wxyz": sensor.quat.tolist()}
    camera = model.camera("stereo_left")
    mount = model.site("cam_left_site")
    sensors["camera_left"] = {
        "pos_m": camera.pos.tolist(), "quat_wxyz": camera.quat.tolist(), "fovy_deg": float(camera.fovy[0]),
        "mount_pos_m": mount.pos.tolist(), "mount_quat_wxyz": mount.quat.tolist(),
    }
    return sensors


def prepare(data: Path, output: Path) -> None:
    files = ("metadata.json", "pwm.csv", "real.csv", "voltage.csv")
    for trial in (*TRAIN, *HOLDOUT):
        for name in files:
            if not (data / trial / name).is_file():
                raise FileNotFoundError(data / trial / name)
    output.mkdir(parents=True, exist_ok=False)
    (output / "model" / "scenes").mkdir(parents=True)
    (output / "config").mkdir()
    scene_name = "bluerov2_dfki_scene.xml"
    source_scene = RUNTIME / "scenes" / scene_name
    shutil.copy2(source_scene, output / "model" / "scenes")
    shutil.copytree(RUNTIME / "scenes" / "bluerov2", output / "model" / "scenes" / "bluerov2")
    shutil.copytree(RUNTIME / "assets" / "bluerov2_standard", output / "model" / "assets" / "bluerov2_standard")
    for name in ("sim_profiles_bluerov2.json", "thruster_performance.json"):
        shutil.copy2(RUNTIME / "config" / name, output / "config")
    for name in (*TRAIN, *HOLDOUT):
        target = output / "trials" / name
        target.mkdir(parents=True)
        for source in files:
            shutil.copy2(data / name / source, target)
    manifest = {
        "version": 1,
        "name": "bluerov2_dfki",
        "scene": f"model/scenes/{scene_name}",
        "profile_file": "config/sim_profiles_bluerov2.json",
        "base_profile": "bluerov2_standard",
        "thruster_perf_file": "config/thruster_performance.json",
        "thrusters": [
            {"name": name, "kind": "horizontal" if name in BLUEROV2_HORIZONTAL_THRUSTERS else "vertical",
             "pwm_sign": pwm, "forward_sign": forward, "spin_sign": spin}
            for name, pwm, forward, spin in zip(
                BLUEROV2_SERVO_MAP, BLUEROV2_DFKI_SERVO_SIGNS, BLUEROV2_T200_FORWARD_SIGNS, BLUEROV2_PROP_SPIN_SIGNS,
            )
        ],
        "sensors": source_sensors(source_scene),
        "train": TRAIN,
        "holdout": HOLDOUT,
        "yaw_target_trial": "random",
        "reference_profiles": {
            "existing": "bluerov2_dfki_calibrated",
            "existing_yaw": "bluerov2_dfki_yaw_experiment",
        },
    }
    write_json(output / "vehicle.json", manifest)
    print(output / "vehicle.json")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dfki-data", required=True, type=Path, help="Parent of normalized DFKI trial directories")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        prepare(args.dfki_data.resolve(), args.output.resolve())
    except (OSError, ValueError) as exc:
        parser.exit(1, f"prepare_bluerov2_real2sim: {exc}\n")
