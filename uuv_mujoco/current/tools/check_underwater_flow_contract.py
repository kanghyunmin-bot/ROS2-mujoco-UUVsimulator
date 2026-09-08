#!/usr/bin/env python3
"""Regression checks for water-relative flow and transient wrench handling."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402
from sim.runtime.control_bridge_real_start_status import (  # noqa: E402
    create_runtime_real_start_status,
)
from sim.physics.fluidcoef_immersion_runtime import FluidcoefImmersionRuntime  # noqa: E402
from sim.runtime.underwater_hydrodynamics_extra import apply_empirical_pitch_lift_heave  # noqa: E402
from sim.runtime.underwater_relative_acceleration import (  # noqa: E402
    reset_relative_acceleration_on_hold_transition,
    update_relative_acceleration,
)
from sim.runtime.hydrodynamics_runtime_current import configure_mujoco_current  # noqa: E402


def _check_added_mass_seed_and_time_reset() -> None:
    runtime = SimpleNamespace(
        hydrodynamics=SimpleNamespace(fossen_residual_added_mass_active=True),
        use_custom_hydrodynamics=False,
        data=SimpleNamespace(time=10.0),
        prev_rel_nu_body=np.zeros(6, dtype=np.float64),
        prev_rel_nu_valid=False,
        prev_rel_sample_time_s=float("nan"),
    )
    initial = np.array([1.0, -0.2, 0.1, 0.0, 0.0, 0.4], dtype=np.float64)
    acceleration = update_relative_acceleration(runtime, initial, 0.005)
    np.testing.assert_array_equal(acceleration, np.zeros(6))

    runtime.data.time = 10.005
    next_sample = initial.copy()
    next_sample[0] += 0.05
    acceleration = update_relative_acceleration(runtime, next_sample, 0.005)
    np.testing.assert_allclose(acceleration, [10.0, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1.0e-12)

    runtime.data.time = 0.0
    reset_acceleration = update_relative_acceleration(runtime, next_sample * 2.0, 0.005)
    np.testing.assert_array_equal(reset_acceleration, np.zeros(6))


def _check_initial_hold_release_resets_added_mass_history() -> None:
    runtime = SimpleNamespace(
        hydrodynamics=SimpleNamespace(fossen_residual_added_mass_active=True),
        use_custom_hydrodynamics=True,
        data=SimpleNamespace(time=2.0),
        prev_rel_nu_body=np.zeros(6, dtype=np.float64),
        prev_rel_nu_valid=True,
        prev_rel_sample_time_s=1.995,
        previous_initial_depth_hold_active=True,
    )
    runtime.prev_rel_nu_body[0] = 0.0
    self_reset = reset_relative_acceleration_on_hold_transition(
        runtime,
        hold_active=False,
    )
    if not self_reset:
        raise AssertionError("hold release did not reset added-mass history")
    runtime.data.time = 2.005
    acceleration = update_relative_acceleration(
        runtime,
        np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        0.005,
    )
    np.testing.assert_array_equal(acceleration, np.zeros(6))


def _check_real_start_keeps_scene_density_after_fluid_owner_switch() -> None:
    initial_state = SimpleNamespace(
        real_start_required=True,
        real_start_depth_tol_m=0.1,
        real_start_attitude_tol_rad=0.1,
        real_start_velocity_tol_mps=0.1,
    )
    status = create_runtime_real_start_status(
        ros_bridge_runtime=SimpleNamespace(get=lambda: None),
        env_float=lambda _name, default: float(default),
        initial_runtime_state=initial_state,
        initial_depth_hold={
            "active": True,
            "release_linear_velocity_body": np.zeros(3),
            "release_angular_velocity_body": np.zeros(3),
        },
        water_surface_z=0.0,
        scene_fluid_density=1000.0,
        base_origin_world=lambda: np.array([0.0, 0.0, -1.0]),
        bar30_depth_now_m=lambda: 1.0,
        data=SimpleNamespace(qpos=np.array([0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0])),
        model=SimpleNamespace(opt=SimpleNamespace(density=0.0)),
        world_qpos_adr=0,
    )
    if status.model_density_fn() != 1000.0:
        raise AssertionError("real-start BAR30 density followed disabled MuJoCo fluid")


def _check_body_frame_yaw_rate_ownership() -> None:
    hyd = SimpleNamespace(
        hydro_pitch_moment_coeff=0.0,
        hydro_vertical_lift_coeff=0.0,
        hydro_vertical_lift_deadband_mps=0.2,
        hydro_vertical_lift_power=2.0,
        hydro_yawrate_heave_pos_coeff=10.0,
        hydro_yawrate_heave_neg_coeff=0.0,
        hydro_yawrate_heave_speed_deadband_mps=0.2,
        hydro_yawrate_heave_yaw_deadband_radps=0.1,
        heave_extra_damping_n_per_mps=0.0,
        cfd_dynamic_wrench_owns_z=False,
    )
    data = SimpleNamespace(
        # Deliberately contradictory: qvel must not own this body-frame term.
        qvel=np.array([0.0, 0.0, 0.0, 0.0, 0.0, -9.0], dtype=np.float64),
        xfrc_applied=np.zeros((1, 6), dtype=np.float64),
    )
    runtime = SimpleNamespace(hydrodynamics=hyd, data=data, base_id=0)
    apply_empirical_pitch_lift_heave(
        runtime,
        base_rot=np.eye(3),
        rel_lin_vel_body=np.array([1.0, 0.0, 0.0]),
        ang_vel_body=np.array([0.0, 0.0, 0.5]),
        buoyancy_submerged=1.0,
    )
    np.testing.assert_allclose(data.xfrc_applied[0, 2], -3.2, atol=1.0e-12)


def _check_buoy_galilean_current_parity() -> None:
    runtime = object.__new__(CourseBuoyRuntime)
    runtime.water_current_world = np.array([0.2, -0.1, 0.0], dtype=np.float64)
    runtime.water_linear_drag_nspm = 0.4
    runtime.water_quadratic_drag_nspm2 = 0.0
    runtime.water_angular_drag_nmsprad = 0.0
    runtime.model = SimpleNamespace(body_mass=np.array([0.01], dtype=np.float64))
    runtime._buoy_center_world = lambda _buoy: np.array([0.0, 0.0, 0.0], dtype=np.float64)
    runtime._touches_float_waterline = lambda _buoy, _center_z: True
    runtime._body_angular_velocity = lambda _body_id: np.zeros(3, dtype=np.float64)
    buoy = SimpleNamespace(body_id=0, free_dofadr=-1, detached=False)

    runtime._body_linear_velocity = lambda _body_id: runtime.water_current_world.copy()
    comoving = runtime._water_drag_wrench(buoy, vehicle_contact=False, dt=0.005)
    np.testing.assert_allclose(comoving, np.zeros(6), atol=1.0e-12)

    runtime._body_linear_velocity = lambda _body_id: np.zeros(3, dtype=np.float64)
    stationary = runtime._water_drag_wrench(buoy, vehicle_contact=False, dt=0.005)
    np.testing.assert_allclose(stationary[:3], [0.08, -0.04, 0.0], atol=1.0e-12)

    model = SimpleNamespace(opt=SimpleNamespace(wind=np.zeros(3, dtype=np.float64)))
    configure_mujoco_current(
        model,
        water_current_world=runtime.water_current_world,
        use_custom_hydrodynamics=False,
        log=lambda _message: None,
    )
    np.testing.assert_allclose(model.opt.wind, runtime.water_current_world, atol=1.0e-12)


def _check_fluidcoef_waterline_continuity() -> None:
    model = SimpleNamespace(
        geom_fluid=np.array([[1.0, 2.0, 4.0, 6.0, 8.0, 10.0]], dtype=np.float64),
        geom_size=np.array([[0.2, 0.1, 0.1]], dtype=np.float64),
        geom_type=np.array([4], dtype=np.int32),
    )
    data = SimpleNamespace(
        geom_xpos=np.array([[0.0, 0.0, 0.0]], dtype=np.float64),
        geom_xmat=np.array([[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]], dtype=np.float64),
    )
    runtime = FluidcoefImmersionRuntime(
        model=model,
        data=data,
        fluid_geom_ids=[0],
        water_surface_z=0.0,
        enabled=True,
        update_unscaled=lambda _linear, _angular: None,
    )
    zero = np.zeros(3, dtype=np.float64)
    runtime.update(zero, zero)
    np.testing.assert_allclose(model.geom_fluid[0, 1:6], [1.0, 2.0, 3.0, 4.0, 5.0])
    # A second update at the same waterline must not compound to 25 percent.
    runtime.update(zero, zero)
    np.testing.assert_allclose(model.geom_fluid[0, 1:6], [1.0, 2.0, 3.0, 4.0, 5.0])

    data.geom_xpos[0, 2] = -1.0
    runtime.update(zero, zero)
    np.testing.assert_allclose(model.geom_fluid[0, 1:6], [2.0, 4.0, 6.0, 8.0, 10.0])
    data.geom_xpos[0, 2] = 1.0
    runtime.update(zero, zero)
    np.testing.assert_allclose(model.geom_fluid[0, 1:6], np.zeros(5), atol=1.0e-12)


def _check_fluidcoef_geom_type_support() -> None:
    # World-Z in local coordinates is (0.6, 0.0, 0.8). The remaining rows do
    # not affect the support calculation but complete a finite 3x3 payload.
    rotation = np.array(
        [0.8, 0.0, -0.6, 0.0, 1.0, 0.0, 0.6, 0.0, 0.8],
        dtype=np.float64,
    )
    model = SimpleNamespace(
        geom_fluid=np.ones((5, 6), dtype=np.float64),
        geom_size=np.array(
            [
                [2.0, 0.0, 0.0],  # sphere
                [2.0, 3.0, 4.0],  # ellipsoid
                [2.0, 3.0, 0.0],  # cylinder: radius, local-Z half-length
                [2.0, 3.0, 0.0],  # capsule: radius, segment half-length
                [2.0, 3.0, 4.0],  # box half-sizes
            ],
            dtype=np.float64,
        ),
        geom_type=np.array([2, 4, 5, 3, 6], dtype=np.int32),
        geom_rbound=np.full(5, 10.0, dtype=np.float64),
    )
    data = SimpleNamespace(
        geom_xpos=np.zeros((5, 3), dtype=np.float64),
        geom_xmat=np.tile(rotation, (5, 1)),
    )
    runtime = FluidcoefImmersionRuntime(
        model=model,
        data=data,
        fluid_geom_ids=range(5),
        water_surface_z=0.0,
        enabled=True,
        update_unscaled=lambda _linear, _angular: None,
    )
    expected = np.array(
        [
            2.0,
            math.sqrt((0.6 * 2.0) ** 2 + (0.8 * 4.0) ** 2),
            2.0 * 0.6 + 3.0 * 0.8,
            2.0 + 3.0 * 0.8,
            2.0 * 0.6 + 4.0 * 0.8,
        ],
        dtype=np.float64,
    )
    actual = np.array([runtime._vertical_half_extent(index) for index in range(5)])
    np.testing.assert_allclose(actual, expected, atol=1.0e-12)


def _check_actual_mjcf_fluid_geom_support() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(ROOT / "scenes" / "tank_current_scene.xml"))
    data = mujoco.MjData(model)
    world_joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint"))
    names = (
        "fluid_center_enclosure",
        "fluid_port_lower_body",
        "fluid_starboard_lower_body",
    )
    geom_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
        for name in names
    ]
    if world_joint_id < 0 or any(geom_id < 0 for geom_id in geom_ids):
        raise AssertionError("competition MJCF fluid geom contract is incomplete")
    world_qpos_adr = int(model.jnt_qposadr[world_joint_id])
    runtime = FluidcoefImmersionRuntime(
        model=model,
        data=data,
        fluid_geom_ids=geom_ids,
        water_surface_z=0.0,
        enabled=True,
        update_unscaled=lambda _linear, _angular: None,
    )

    center_radius, center_half_length = (float(value) for value in model.geom_size[geom_ids[0], :2])
    lower_radius, lower_half_length = (float(value) for value in model.geom_size[geom_ids[1], :2])
    for pitch_deg in (0.0, 30.0, 90.0):
        pitch_rad = math.radians(pitch_deg)
        data.qpos[world_qpos_adr + 3 : world_qpos_adr + 7] = [
            math.cos(0.5 * pitch_rad),
            0.0,
            math.sin(0.5 * pitch_rad),
            0.0,
        ]
        mujoco.mj_forward(model, data)
        axis_vertical = abs(math.sin(pitch_rad))
        radial_vertical = abs(math.cos(pitch_rad))
        expected_center = center_radius * radial_vertical + center_half_length * axis_vertical
        expected_lower = lower_radius + lower_half_length * axis_vertical
        actual = [runtime._vertical_half_extent(geom_id) for geom_id in geom_ids]
        np.testing.assert_allclose(
            actual,
            [expected_center, expected_lower, expected_lower],
            atol=1.0e-10,
            err_msg=f"competition fluid geom support mismatch at pitch={pitch_deg:.0f}deg",
        )


def main() -> int:
    _check_added_mass_seed_and_time_reset()
    _check_initial_hold_release_resets_added_mass_history()
    _check_real_start_keeps_scene_density_after_fluid_owner_switch()
    _check_body_frame_yaw_rate_ownership()
    _check_buoy_galilean_current_parity()
    _check_fluidcoef_waterline_continuity()
    _check_fluidcoef_geom_type_support()
    _check_actual_mjcf_fluid_geom_support()
    print("underwater_flow_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
