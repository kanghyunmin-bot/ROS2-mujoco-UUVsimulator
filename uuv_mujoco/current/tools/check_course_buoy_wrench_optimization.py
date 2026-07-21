#!/usr/bin/env python3
"""Check exact buoyancy-wrench parity and report the hot-path speedup."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import mujoco
import numpy as np


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from sim.runtime.course_buoy_runtime import CourseBuoy, CourseBuoyRuntime  # noqa: E402
from sim.runtime.env import env_flag, env_float  # noqa: E402


def _runtime(scene: Path) -> CourseBuoyRuntime:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=0.0,
        env_float=env_float,
        env_flag=env_flag,
        log=lambda _message: None,
    )


def _reference_body_weight(runtime: CourseBuoyRuntime, buoy: CourseBuoy) -> float:
    gravity_z = float(np.asarray(runtime.model.opt.gravity, dtype=np.float64)[2])
    return float(runtime.model.body_mass[buoy.body_id]) * abs(gravity_z)


def _reference_surface_target(runtime: CourseBuoyRuntime, buoy: CourseBuoy) -> float:
    if runtime.buoyancy_n <= 0.0:
        return runtime.water_surface_z - runtime.float_half_height_m
    weight_n = _reference_body_weight(runtime, buoy)
    full_upthrust_n = weight_n + runtime.buoyancy_n
    equilibrium_fraction = float(np.clip(weight_n / full_upthrust_n, 0.0, 1.0))
    return (
        runtime.water_surface_z
        + runtime.float_half_height_m
        - 2.0 * runtime.float_half_height_m * equilibrium_fraction
    )


def _reference_release_scale(runtime: CourseBuoyRuntime, buoy: CourseBuoy) -> float:
    if not buoy.detached or buoy.release_time_s < 0.0 or runtime.release_stabilize_s <= 0.0:
        return 1.0
    elapsed_s = float(runtime.data.time) - float(buoy.release_time_s)
    if elapsed_s < 0.0 or elapsed_s >= runtime.release_stabilize_s:
        return 1.0
    progress = elapsed_s / max(runtime.release_stabilize_s, 1.0e-9)
    return float(0.35 + 0.65 * np.clip(progress, 0.0, 1.0))


def _reference_float_wrench(
    runtime: CourseBuoyRuntime,
    buoy: CourseBuoy,
    *,
    vehicle_contact: bool,
) -> np.ndarray:
    """The pre-optimization implementation, retained only as a test oracle."""

    wrench = np.zeros(6, dtype=np.float64)
    if runtime.buoyancy_n <= 0.0:
        return wrench
    center_z = float(runtime._buoy_center_world(buoy)[2])
    if not runtime._touches_float_waterline(buoy, center_z):
        return wrench
    velocity_z = float(runtime._body_linear_velocity(buoy.body_id)[2])
    target_z = _reference_surface_target(runtime, buoy)
    neutral_upthrust_n = _reference_body_weight(runtime, buoy)
    reserve_buoyancy_n = runtime.buoyancy_n if buoy.detached else min(runtime.buoyancy_n, 0.98)
    buoyancy_cap_n = reserve_buoyancy_n * _reference_release_scale(runtime, buoy)
    if vehicle_contact and center_z < target_z:
        wrench[2] = neutral_upthrust_n + buoyancy_cap_n
    else:
        depth_error_m = target_z - center_z
        net_lift_n = (
            runtime.surface_spring_npm * depth_error_m
            - runtime.vertical_damping_nspm * velocity_z
        )
        wrench[2] = neutral_upthrust_n + float(np.clip(net_lift_n, 0.0, buoyancy_cap_n))
    if buoy.cob_site_id >= 0:
        center_of_mass = np.asarray(runtime.data.xipos[buoy.body_id], dtype=np.float64)
        center_of_buoyancy = np.asarray(runtime.data.site_xpos[buoy.cob_site_id], dtype=np.float64)
        wrench[3:6] += np.cross(center_of_buoyancy - center_of_mass, wrench[0:3])
    return wrench


def _check_randomized_parity(runtime: CourseBuoyRuntime, samples_per_buoy: int) -> tuple[int, float]:
    rng = np.random.default_rng(260717)
    cases = 0
    max_abs = 0.0
    for buoy in runtime.buoys:
        reference_weight = _reference_body_weight(runtime, buoy)
        reference_target = _reference_surface_target(runtime, buoy)
        if buoy.cached_body_weight_n != reference_weight:
            raise AssertionError(f"cached weight mismatch for {buoy.name}")
        if buoy.cached_surface_target_center_z != reference_target:
            raise AssertionError(f"cached target mismatch for {buoy.name}")
        for sample_index in range(samples_per_buoy):
            qpos = buoy.free_qposadr
            dof = buoy.free_dofadr
            runtime.data.qpos[qpos : qpos + 3] = [
                rng.uniform(-5.0, 5.0),
                rng.uniform(-5.0, 5.0),
                rng.uniform(-9.0, 0.15),
            ]
            quaternion = rng.normal(size=4)
            quaternion /= max(float(np.linalg.norm(quaternion)), 1.0e-12)
            runtime.data.qpos[qpos + 3 : qpos + 7] = quaternion
            runtime.data.qvel[dof : dof + 6] = rng.uniform(-1.5, 1.5, size=6)
            runtime.data.time = float(sample_index) * 0.031
            buoy.detached = bool(sample_index % 2)
            buoy.release_time_s = (
                float(runtime.data.time) - 0.5 * runtime.release_stabilize_s
                if sample_index % 3 == 1
                else -1.0
            )
            mujoco.mj_forward(runtime.model, runtime.data)
            for vehicle_contact in (False, True):
                reference = _reference_float_wrench(
                    runtime,
                    buoy,
                    vehicle_contact=vehicle_contact,
                )
                optimized = runtime._float_buoyancy_wrench(
                    buoy,
                    vehicle_contact=vehicle_contact,
                )
                difference = float(np.max(np.abs(reference - optimized)))
                max_abs = max(max_abs, difference)
                cases += 1
                if not np.array_equal(reference, optimized):
                    raise AssertionError(
                        f"wrench mismatch for {buoy.name} sample={sample_index} "
                        f"contact={vehicle_contact}: max_abs={difference:.3e}"
                    )
    return cases, max_abs


def _reference_clear_persisted_runtime_wrench(
    runtime: CourseBuoyRuntime,
    buoy: CourseBuoy,
) -> None:
    """The pre-optimization NumPy-mask implementation used as an oracle."""

    last = buoy.last_runtime_wrench
    if not np.any(last):
        return
    current = np.array(runtime.data.xfrc_applied[buoy.body_id, :], dtype=np.float64)
    active = np.abs(last) > 1.0e-9
    if not bool(np.any(active)):
        buoy.last_runtime_wrench[:] = 0.0
        return
    same_direction = np.sign(current[active]) == np.sign(last[active])
    still_present = np.abs(current[active]) >= 0.5 * np.abs(last[active])
    removable = active.copy()
    removable[active] = same_direction & still_present
    if bool(np.any(removable)):
        runtime.data.xfrc_applied[buoy.body_id, removable] -= last[removable]
    buoy.last_runtime_wrench[:] = 0.0


def _reference_apply(runtime: CourseBuoyRuntime, dt: float) -> None:
    """The allocating pre-optimization full apply path retained as an oracle."""

    if not runtime.buoys:
        return
    logic_due = runtime._update_due()
    contacted_buoy_bodies = runtime._vehicle_contacted_buoy_body_ids()
    rake_contacted_buoy_bodies = runtime._release_probe_contacted_buoy_body_ids()
    for buoy in runtime.buoys:
        _reference_clear_persisted_runtime_wrench(runtime, buoy)
        vehicle_contact = buoy.body_id in contacted_buoy_bodies
        was_detached = buoy.detached
        rake_contact = buoy.body_id in rake_contacted_buoy_bodies
        release_contact = rake_contact or runtime._has_release_probe_proximity(buoy)
        if release_contact:
            buoy.last_vehicle_contact_time_s = float(getattr(runtime.data, "time", 0.0))
        else:
            runtime._reset_contact_release_sample(buoy)
        if buoy.has_magnet and not buoy.detached:
            runtime._release_if_contact_or_break_force(buoy, vehicle_contact=release_contact)
        if buoy.detached and was_detached:
            runtime._stabilize_released_buoy(buoy, vehicle_contact=vehicle_contact)
            runtime._restore_released_buoy_collisions_after_grace(buoy)

        wrench = np.zeros(6, dtype=np.float64)
        wrench += runtime._float_buoyancy_wrench(buoy, vehicle_contact=vehicle_contact)
        wrench += runtime._water_drag_wrench(buoy, vehicle_contact=vehicle_contact, dt=dt)
        wrench += runtime._collector_net_wrench(buoy)
        if buoy.has_magnet and not buoy.detached:
            wrench += runtime._magnet_hold_wrench(buoy, dt)
        runtime.data.xfrc_applied[buoy.body_id, :] += wrench
        buoy.last_runtime_wrench = wrench
        runtime._apply_surface_float_guard(buoy, vehicle_contact=vehicle_contact)
    if logic_due:
        runtime._write_tracking_sample()


def _max_exact_difference(reference: np.ndarray, optimized: np.ndarray, label: str) -> float:
    if np.array_equal(reference, optimized):
        return 0.0
    difference = (
        float(
            np.max(
                np.abs(
                    np.asarray(reference, dtype=np.float64)
                    - np.asarray(optimized, dtype=np.float64)
                )
            )
        )
        if reference.size
        else 0.0
    )
    raise AssertionError(f"full apply {label} mismatch: max_abs={difference:.3e}")


def _check_full_apply_parity(scene: Path, steps: int) -> tuple[int, float]:
    """Compare complete state evolution, including an external-force release."""

    reference = _runtime(scene)
    optimized = _runtime(scene)
    scan_counts = {"snapshot": 0}
    optimized_contact_snapshot = optimized._contact_snapshot

    def counted_contact_snapshot():
        scan_counts["snapshot"] += 1
        return optimized_contact_snapshot()

    optimized._contact_snapshot = counted_contact_snapshot  # type: ignore[method-assign]

    reference_buoy = next(item for item in reference.buoys if item.name == "course_buoy_a_yellow_1")
    optimized_buoy = next(item for item in optimized.buoys if item.name == "course_buoy_a_yellow_1")
    break_step = min(20, max(0, steps // 4))
    max_abs = 0.0
    for step in range(steps):
        if step == break_step:
            reference.data.xfrc_applied[reference_buoy.body_id, 2] += 16.0
            optimized.data.xfrc_applied[optimized_buoy.body_id, 2] += 16.0

        _reference_apply(reference, 0.005)
        optimized.apply(0.005)

        if step == break_step:
            reference.data.xfrc_applied[reference_buoy.body_id, 2] -= 16.0
            optimized.data.xfrc_applied[optimized_buoy.body_id, 2] -= 16.0

        for label, reference_values, optimized_values in (
            ("qpos/pre-step", reference.data.qpos, optimized.data.qpos),
            ("qvel/pre-step", reference.data.qvel, optimized.data.qvel),
            ("xfrc/pre-step", reference.data.xfrc_applied, optimized.data.xfrc_applied),
            ("eq_active/pre-step", reference.data.eq_active, optimized.data.eq_active),
            ("geom_contype", reference.model.geom_contype, optimized.model.geom_contype),
            ("geom_conaffinity", reference.model.geom_conaffinity, optimized.model.geom_conaffinity),
        ):
            max_abs = max(
                max_abs,
                _max_exact_difference(reference_values, optimized_values, label),
            )
        for reference_item, optimized_item in zip(reference.buoys, optimized.buoys):
            max_abs = max(
                max_abs,
                _max_exact_difference(
                    reference_item.last_runtime_wrench,
                    optimized_item.last_runtime_wrench,
                    f"{reference_item.name}/last_runtime_wrench",
                ),
            )
        if reference.status_by_name() != optimized.status_by_name():
            raise AssertionError(f"full apply status mismatch at step {step}")

        mujoco.mj_step(reference.model, reference.data)
        mujoco.mj_step(optimized.model, optimized.data)
        max_abs = max(
            max_abs,
            _max_exact_difference(reference.data.qpos, optimized.data.qpos, "qpos/post-step"),
            _max_exact_difference(reference.data.qvel, optimized.data.qvel, "qvel/post-step"),
        )

    if not reference_buoy.detached or not optimized_buoy.detached:
        raise AssertionError("16 N external force did not detach the parity-test buoy")
    if scan_counts != {"snapshot": steps}:
        raise AssertionError(f"single-pass contact snapshot was not run every physics step: {scan_counts}")
    return steps, max_abs


def _benchmark(runtime: CourseBuoyRuntime, iterations: int) -> tuple[float, float]:
    # Restore a representative stable scene. Both paths read the same state and
    # return new arrays, so timing them does not mutate the physics contract.
    mujoco.mj_resetData(runtime.model, runtime.data)
    mujoco.mj_forward(runtime.model, runtime.data)
    for buoy in runtime.buoys:
        buoy.detached = not buoy.has_magnet
        buoy.release_time_s = -1.0

    def reference_batch() -> None:
        for item in runtime.buoys:
            _reference_float_wrench(runtime, item, vehicle_contact=False)

    def optimized_batch() -> None:
        for item in runtime.buoys:
            runtime._float_buoyancy_wrench(item, vehicle_contact=False)

    for _ in range(30):
        reference_batch()
        optimized_batch()
    reference_started = time.perf_counter()
    for _ in range(iterations):
        reference_batch()
    reference_ms = 1000.0 * (time.perf_counter() - reference_started) / iterations
    optimized_started = time.perf_counter()
    for _ in range(iterations):
        optimized_batch()
    optimized_ms = 1000.0 * (time.perf_counter() - optimized_started) / iterations
    return reference_ms, optimized_ms


def _benchmark_full_apply(scene: Path, iterations: int) -> tuple[float, float]:
    def measure(runtime: CourseBuoyRuntime, apply_once) -> float:
        for _ in range(20):
            apply_once(runtime, 0.005)
            mujoco.mj_step(runtime.model, runtime.data)
        elapsed_s = 0.0
        for _ in range(iterations):
            started = time.perf_counter()
            apply_once(runtime, 0.005)
            elapsed_s += time.perf_counter() - started
            mujoco.mj_step(runtime.model, runtime.data)
        return 1000.0 * elapsed_s / iterations

    reference = _runtime(scene)
    optimized = _runtime(scene)
    return measure(reference, _reference_apply), measure(
        optimized,
        lambda runtime, dt: runtime.apply(dt),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=CURRENT_DIR / "scenes" / "tank_current_scene.xml",
    )
    parser.add_argument("--samples-per-buoy", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--full-steps", type=int, default=600)
    args = parser.parse_args()
    scene = args.scene.resolve()
    runtime = _runtime(scene)
    cases, max_abs = _check_randomized_parity(runtime, max(1, args.samples_per_buoy))
    reference_ms, optimized_ms = _benchmark(runtime, max(10, args.iterations))
    speedup = reference_ms / max(optimized_ms, 1.0e-12)
    full_steps, full_max_abs = _check_full_apply_parity(scene, max(1, args.full_steps))
    full_reference_ms, full_optimized_ms = _benchmark_full_apply(
        scene,
        max(10, args.iterations),
    )
    full_speedup = full_reference_ms / max(full_optimized_ms, 1.0e-12)
    print(
        "course buoy wrench optimization: PASS "
        f"cases={cases} max_abs={max_abs:.3e} "
        f"reference={reference_ms:.3f}ms optimized={optimized_ms:.3f}ms "
        f"speedup={speedup:.2f}x "
        f"full_steps={full_steps} full_max_abs={full_max_abs:.3e} "
        f"full_reference={full_reference_ms:.3f}ms "
        f"full_optimized={full_optimized_ms:.3f}ms "
        f"full_speedup={full_speedup:.2f}x"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
