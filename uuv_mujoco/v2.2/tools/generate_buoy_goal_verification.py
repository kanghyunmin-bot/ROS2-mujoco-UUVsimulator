#!/usr/bin/env python3
"""Generate repeatable evidence for course-buoy float and collector behavior."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
DEFAULT_OUT_DIR = REPO / "outputs" / "buoy_goal_verification"
sys.path.insert(0, str(ROOT / "tools"))

from check_buoy_collector_capture import (  # noqa: E402
    body_id,
    joint_id,
    make_runtime,
    place_free_buoy,
    world_from_base_local,
)


BUOY_CASES = (
    ("course_buoy_a_red_1", False),
    ("course_buoy_a_yellow_1", True),
    ("course_buoy_a_orange_1", True),
    ("course_buoy_pinger_white_1", True),
)


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _runtime_buoy(runtime, name: str):
    return next(item for item in runtime.buoys if item.name == name)


def _detach_with_force(mujoco, model, data, runtime, buoy_body: int) -> None:
    data.xfrc_applied[buoy_body, 2] += 16.0
    runtime.apply(float(model.opt.timestep))
    data.xfrc_applied[buoy_body, 2] -= 16.0
    mujoco.mj_forward(model, data)


def sample_buoy_case(mujoco, model, name: str, detach: bool) -> tuple[list[dict[str, float | int | str]], dict[str, float | int | str]]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    runtime_buoy = _runtime_buoy(runtime, name)
    buoy_body = body_id(mujoco, model, f"{name}_float")
    buoy_joint = joint_id(mujoco, model, f"{name}_free")
    dofadr = int(model.jnt_dofadr[buoy_joint])
    target_z = runtime._surface_target_center_z(runtime_buoy)
    start_z = float(data.xipos[buoy_body, 2])

    if detach:
        _detach_with_force(mujoco, model, data, runtime, buoy_body)

    rows: list[dict[str, float | int | str]] = []
    timestep = float(model.opt.timestep)
    sample_stride = max(1, round(0.1 / timestep))
    steps = round(32.0 / timestep)
    for step in range(steps + 1):
        if step % sample_stride == 0:
            rows.append(
                {
                    "case": name,
                    "t_s": round(step * timestep, 3),
                    "z_m": float(data.xipos[buoy_body, 2]),
                    "vz_mps": float(data.qvel[dofadr + 2]),
                    "target_z_m": target_z,
                    "detached": int(runtime_buoy.detached),
                }
            )
        runtime.apply(timestep)
        mujoco.mj_step(model, data)

    summary = {
        "case": name,
        "has_magnet": int(runtime_buoy.has_magnet),
        "detached": int(runtime_buoy.detached),
        "release_state": "released" if runtime_buoy.has_magnet and runtime_buoy.detached else ("attached" if runtime_buoy.has_magnet else "free_surface"),
        "start_z_m": start_z,
        "final_z_m": float(data.xipos[buoy_body, 2]),
        "target_z_m": target_z,
        "final_vz_mps": float(data.qvel[dofadr + 2]),
        "weight_n": runtime._body_weight_n(runtime_buoy),
        "full_upthrust_n": runtime._full_immersion_upthrust_n(runtime_buoy),
        "net_full_lift_n": runtime._full_immersion_upthrust_n(runtime_buoy) - runtime._body_weight_n(runtime_buoy),
    }
    return rows, summary


def _collector_contact_count(mujoco, model, data, buoy_geom_name: str) -> int:
    count = 0
    buoy_geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, buoy_geom_name))
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.geom1) == buoy_geom_id or int(contact.geom2) == buoy_geom_id:
            count += 1
    return count


def sample_collector_tow(mujoco, model) -> tuple[list[dict[str, float | int]], dict[str, float | int]]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = make_runtime(mujoco, model, data)
    base = body_id(mujoco, model, "base_link")
    buoy = body_id(mujoco, model, "course_buoy_a_red_1_float")
    world_joint = joint_id(mujoco, model, "world_joint")
    base_qposadr = int(model.jnt_qposadr[world_joint])
    base_dofadr = int(model.jnt_dofadr[world_joint])
    start_base = np.array(data.xpos[base], dtype=np.float64)

    place_free_buoy(
        mujoco,
        model,
        data,
        buoy,
        "course_buoy_a_red_1",
        world_from_base_local(data, base, np.array([-0.135, 0.0, 0.501], dtype=np.float64)),
    )
    start_buoy = np.array(data.xipos[buoy], dtype=np.float64)

    rows: list[dict[str, float | int]] = []
    timestep = float(model.opt.timestep)
    sample_stride = max(1, round(0.1 / timestep))
    contact_samples = 0
    steps = round(3.0 / timestep)
    for step in range(steps + 1):
        if _collector_contact_count(mujoco, model, data, "course_buoy_a_red_1_float_geom") > 0:
            contact_samples += 1
        if step % sample_stride == 0:
            rows.append(
                {
                    "t_s": round(step * timestep, 3),
                    "base_dx_m": float(data.xpos[base, 0] - start_base[0]),
                    "red_dx_m": float(data.xipos[buoy, 0] - start_buoy[0]),
                    "red_z_m": float(data.xipos[buoy, 2]),
                    "contact_samples": contact_samples,
                }
            )
        data.qvel[base_dofadr + 0] = 0.35
        data.qvel[base_dofadr + 1] = 0.0
        data.qvel[base_dofadr + 2] = 0.0
        data.qvel[base_dofadr + 3 : base_dofadr + 6] = 0.0
        runtime.apply(timestep)
        mujoco.mj_step(model, data)
        data.qpos[base_qposadr + 1] = start_base[1]
        data.qpos[base_qposadr + 2] = start_base[2]
        data.qpos[base_qposadr + 3 : base_qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

    summary = {
        "base_dx_m": float(data.xpos[base, 0] - start_base[0]),
        "red_dx_m": float(data.xipos[buoy, 0] - start_buoy[0]),
        "red_final_z_m": float(data.xipos[buoy, 2]),
        "contact_samples": contact_samples,
    }
    return rows, summary


def plot_outputs(out_dir: Path, buoy_rows: list[dict[str, float | int | str]], collector_rows: list[dict[str, float | int]]) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)

    for name, _detach in BUOY_CASES:
        rows = [row for row in buoy_rows if row["case"] == name]
        axes[0].plot([row["t_s"] for row in rows], [row["z_m"] for row in rows], label=name.replace("course_buoy_", ""))
    axes[0].axhline(0.0, color="black", linewidth=0.8, linestyle="--", label="water surface")
    axes[0].set_title("Buoy vertical motion")
    axes[0].set_xlabel("time [s]")
    axes[0].set_ylabel("body center z [m]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot([row["t_s"] for row in collector_rows], [row["base_dx_m"] for row in collector_rows], label="collector base dx")
    axes[1].plot([row["t_s"] for row in collector_rows], [row["red_dx_m"] for row in collector_rows], label="red buoy dx")
    axes[1].set_title("Physical collector tow")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("x displacement [m]")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    path = out_dir / "buoy_goal_verification.png"
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args()

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    buoy_rows: list[dict[str, float | int | str]] = []
    buoy_summaries: list[dict[str, float | int | str]] = []
    for name, detach in BUOY_CASES:
        rows, summary = sample_buoy_case(mujoco, model, name, detach)
        buoy_rows.extend(rows)
        buoy_summaries.append(summary)

    collector_rows, collector_summary = sample_collector_tow(mujoco, model)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.out_dir / "buoy_z_timeseries.csv", buoy_rows)
    _write_csv(args.out_dir / "collector_tow_timeseries.csv", collector_rows)
    plot_path = plot_outputs(args.out_dir, buoy_rows, collector_rows)

    print("BUOY_GOAL_SUMMARY")
    for summary in buoy_summaries:
        print(
            "{case} state={release_state} has_magnet={has_magnet} start_z={start_z_m:.3f} final_z={final_z_m:.3f} "
            "target_z={target_z_m:.3f} final_vz={final_vz_mps:.4f} "
            "weight={weight_n:.3f}N full_upthrust={full_upthrust_n:.3f}N net_full={net_full_lift_n:.3f}N".format(**summary)
        )
    print(
        "collector base_dx={base_dx_m:.3f} red_dx={red_dx_m:.3f} red_final_z={red_final_z_m:.3f} "
        "contact_samples={contact_samples}".format(**collector_summary)
    )
    print(f"outputs={args.out_dir}")
    print(f"plot={plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
