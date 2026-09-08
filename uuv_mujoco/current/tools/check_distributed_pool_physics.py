#!/usr/bin/env python3
"""Actual-MuJoCo smoke for the distributed research-pool physics profile."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from check_research_pool_physics import DT_S, _build_runtime  # noqa: E402


def main() -> int:
    import mujoco

    runtime = _build_runtime(
        mujoco,
        profile_name="research_pool_distributed",
        fluid_model="legacy",
        use_custom_hydrodynamics=True,
    )
    model = runtime.model
    data = runtime.data
    hyd = runtime.hydrodynamics
    underwater = runtime.underwater

    if float(model.opt.density) != 0.0 or float(model.opt.viscosity) != 0.0:
        raise AssertionError("MuJoCo built-in fluid must be disabled for distributed ownership")
    if not hyd.distributed_hydrodynamics.active:
        raise AssertionError("distributed hull model is not active")
    if not hyd.full_matrix_hydrodynamics.active:
        raise AssertionError("full 6x6 added-mass model is not active")
    if not hyd.free_surface.active or hyd.free_surface.mode != "flat":
        raise AssertionError("shared flat free surface is not active")
    if hyd.state_coefficient_scaler.active:
        raise AssertionError("legacy state scaler must be off in the distributed profile")

    underwater.apply(DT_S)
    result = underwater.last_distributed_hydrodynamics_result
    if result is None or result.positions_world_m.shape != (105, 3):
        raise AssertionError("all 105 expanded distributed points did not reach the runtime")
    if not np.all(np.isfinite(data.xfrc_applied[runtime.state.base_id])):
        raise AssertionError("initial distributed wrench is non-finite")
    buoyancy_n = float(np.sum(result.buoyancy_forces_world_n[:, 2]))
    expected_buoyancy_n = 1000.0 * 9.81 * 0.015008
    if not np.isclose(buoyancy_n, expected_buoyancy_n, rtol=0.0, atol=1.0e-9):
        raise AssertionError(
            f"fully submerged buoyancy mismatch: {buoyancy_n} != {expected_buoyancy_n}"
        )
    initial_com = data.xipos[int(runtime.state.base_id)]
    buoyancy_torque = np.sum(
        np.cross(
            result.positions_world_m - initial_com[None, :],
            result.buoyancy_forces_world_n,
        ),
        axis=0,
    )
    if abs(float(buoyancy_torque[1])) > 0.01:
        raise AssertionError(
            f"level-trim buoyancy pitch moment is too large: {buoyancy_torque[1]} Nm"
        )
    if np.allclose(result.currents_world_mps, result.currents_world_mps[0]):
        raise AssertionError("patches did not sample the spatial current independently")

    # MuJoCo reports body linear velocity at the inertial centre. Verify that
    # the runtime converts it to the body origin before applying omega x r to
    # each configured patch.
    qvel = int(runtime.state.world_qvel_adr)
    data.qvel[qvel : qvel + 6] = 0.0
    data.qvel[qvel + 5] = 1.0
    mujoco.mj_forward(model, data)
    underwater.prev_rel_nu_valid = False
    underwater.apply(DT_S)
    rotating = underwater.last_distributed_hydrodynamics_result
    velocity_world = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(
        model,
        data,
        mujoco.mjtObj.mjOBJ_BODY,
        int(runtime.state.base_id),
        velocity_world,
        0,
    )
    com_world = data.xipos[int(runtime.state.base_id)]
    expected_point_velocities = velocity_world[3:][None, :] + np.cross(
        velocity_world[:3][None, :],
        rotating.positions_world_m - com_world[None, :],
    )
    np.testing.assert_allclose(
        rotating.point_velocities_world_mps,
        expected_point_velocities,
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        rotating.wrench_reference_position_world_m,
        com_world,
        atol=1.0e-12,
    )
    rotating_drag = rotating.form_drag_forces_world_n + rotating.skin_drag_forces_world_n
    yaw_drag_torque = float(
        np.sum(
            np.cross(
                rotating.positions_world_m - com_world[None, :],
                rotating_drag,
            ),
            axis=0,
        )[2]
    )
    if yaw_drag_torque >= -0.2:
        raise AssertionError(
            f"expanded quadrature under-resolved yaw drag: {yaw_drag_torque} Nm"
        )

    # A forward body velocity must generate opposing distributed drag. Reset
    # the acceleration history so this assertion isolates drag from added mass.
    data.qvel[qvel : qvel + 6] = 0.0
    data.qvel[qvel] = 0.5
    mujoco.mj_forward(model, data)
    underwater.prev_rel_nu_valid = False
    underwater.apply(DT_S)
    moving = underwater.last_distributed_hydrodynamics_result
    drag_world = np.sum(
        moving.form_drag_forces_world_n + moving.skin_drag_forces_world_n,
        axis=0,
    )
    if float(drag_world[0]) >= 0.0:
        raise AssertionError(f"forward drag must oppose motion, got {drag_world}")

    # Exercise the production thruster construction/update path, including the
    # shared local-current and free-surface samplers used by the main runner.
    thruster = runtime.thruster_actuator
    environment = hyd.water_environment_runtime
    if thruster.current_velocity_sampler is None or thruster.surface_height_sampler is None:
        raise AssertionError("production thruster runtime did not bind the shared water environment")
    for name in thruster.all_thruster_names:
        thruster.target[name] = 0.5
    if thruster.thruster_global["inflow_enabled"]:
        raise AssertionError("uncalibrated thruster inflow must default to disabled")
    thruster.update_forces(DT_S, base_id=int(runtime.state.base_id))
    np.testing.assert_allclose(list(thruster.last_inflow_multiplier.values()), 1.0)
    # Explicitly enable the optional prior to verify its runtime plumbing.
    thruster.thruster_global["inflow_enabled"] = True
    thruster.update_forces(DT_S, base_id=int(runtime.state.base_id))
    inflow_multipliers = np.asarray(
        [thruster.last_inflow_multiplier[name] for name in thruster.all_thruster_names],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(inflow_multipliers)):
        raise AssertionError(f"thruster inflow produced non-finite multipliers: {inflow_multipliers}")
    if np.allclose(inflow_multipliers, 1.0, atol=1.0e-6):
        raise AssertionError("local axial inflow did not reach the production thruster path")
    thruster.thruster_global["inflow_enabled"] = False
    first_site = int(thruster.site_ids[thruster.all_thruster_names[0]])
    first_position = data.site_xpos[first_site].copy()
    np.testing.assert_allclose(
        thruster.current_velocity_sampler(first_position, float(data.time)),
        environment.velocity_world(first_position, float(data.time)),
        atol=1.0e-12,
    )
    if not np.isclose(
        thruster.surface_height_sampler(first_position, float(data.time)),
        environment.surface_height_world_m(first_position, float(data.time)),
        atol=1.0e-12,
    ):
        raise AssertionError("thruster immersion did not use the shared free surface")

    for _index in range(100):
        underwater.apply(DT_S)
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise AssertionError("distributed pool physics produced a non-finite state")
    if not np.all(np.isfinite(underwater.last_full_matrix_wrench_body)):
        raise AssertionError("full-matrix wrench became non-finite")

    hybrid_runtime = _build_runtime(
        mujoco,
        profile_name="research_pool_distributed_hybrid",
        fluid_model="legacy",
        use_custom_hydrodynamics=True,
    )
    hybrid_model = hybrid_runtime.hydrodynamics.distributed_hydrodynamics
    if hybrid_model.config.calibration_status != "stonefish_ellipsoid_hybrid_prior":
        raise AssertionError("hybrid calibration overlay did not reach the runtime")
    volume_shares = hybrid_model.config.volume_shares_m3
    buoyancy_center_body = np.sum(
        hybrid_model.config.positions_body_m * volume_shares[:, None],
        axis=0,
    ) / np.sum(volume_shares)
    np.testing.assert_allclose(
        buoyancy_center_body,
        [-0.0044004264, 0.0, 0.03474765],
        atol=1.0e-9,
    )
    expected_drag_n = np.array([32.9595995748, 71.5639445197, 73.2901428997])
    measured_drag_n = np.zeros(3, dtype=np.float64)
    for axis in range(3):
        velocity = np.zeros(3, dtype=np.float64)
        velocity[axis] = 0.5
        hybrid_result = hybrid_model.evaluate(
            body_position_world_m=np.array([0.0, 0.0, -1.2]),
            rotation_world_from_body=np.eye(3),
            linear_velocity_world_mps=velocity,
            angular_velocity_world_radps=np.zeros(3),
            current_world_mps=np.zeros(3),
            surface_height_world_m=0.0,
            wrench_reference_position_body_m=np.array([-0.0044, 0.0, -0.0452]),
        )
        hybrid_drag = np.sum(
            hybrid_result.form_drag_forces_world_n
            + hybrid_result.skin_drag_forces_world_n,
            axis=0,
        )
        measured_drag_n[axis] = -hybrid_drag[axis]
        residual = hybrid_result.residual_damping_wrench_body
        np.testing.assert_array_equal(
            residual,
            np.zeros(6),
            err_msg="unidentified hybrid residual damping must remain disabled",
        )
        np.testing.assert_allclose(
            hybrid_result.force_world_n,
            np.sum(hybrid_result.patch_forces_world_n, axis=0)
            + hybrid_runtime.data.xmat[int(hybrid_runtime.state.base_id)].reshape(3, 3) @ residual[:3],
            atol=1.0e-10,
        )
    np.testing.assert_allclose(measured_drag_n, expected_drag_n, atol=1.0e-8)
    for _index in range(25):
        hybrid_runtime.underwater.apply(DT_S)
        mujoco.mj_step(hybrid_runtime.model, hybrid_runtime.data)
    if not np.all(np.isfinite(hybrid_runtime.data.qpos)):
        raise AssertionError("hybrid distributed profile produced a non-finite state")

    wave_runtime = _build_runtime(
        mujoco,
        profile_name="research_pool_distributed_waves",
        fluid_model="legacy",
        use_custom_hydrodynamics=True,
    )
    wave_environment = wave_runtime.hydrodynamics.water_environment_runtime
    query = wave_runtime.state.base_origin_world().copy()
    surface_height = wave_environment.surface_height_world_m(query, 0.37)
    wave_velocity = wave_environment.velocity_world(query, 0.37)
    current_only = wave_runtime.hydrodynamics.current_field_runtime.field.velocity_world(
        query,
        0.37,
    )
    if np.isclose(surface_height, 0.0) or np.allclose(wave_velocity, current_only):
        raise AssertionError("harmonic surface geometry/orbital velocity did not reach the runtime")
    for _index in range(25):
        wave_runtime.underwater.apply(DT_S)
        mujoco.mj_step(wave_runtime.model, wave_runtime.data)
    if not np.all(np.isfinite(wave_runtime.data.qpos)):
        raise AssertionError("wave-enabled distributed profile produced non-finite state")

    print(
        "distributed_pool_physics=PASS "
        f"patches={result.positions_world_m.shape[0]} "
        f"buoyancy={buoyancy_n:.3f}N "
        f"forward_drag={drag_world[0]:.3f}N "
        f"yaw_drag={yaw_drag_torque:.3f}Nm "
        f"inflow=[{inflow_multipliers.min():.3f},{inflow_multipliers.max():.3f}] "
        f"hybrid_drag={measured_drag_n.round(3).tolist()}N "
        f"wave_height={surface_height:.4f}m steps=100+25+25 "
        "calibration=stonefish_ellipsoid_hybrid_prior"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
