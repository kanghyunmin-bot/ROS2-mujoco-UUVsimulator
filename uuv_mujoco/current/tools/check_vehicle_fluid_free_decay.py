#!/usr/bin/env python3
"""Low-load competition-scene checks for the vehicle fluid plant.

Scope
-----
This check loads the real competition MJCF and the resolved ``current``
simulation profile.  It exercises the production model setup, distributed
vehicle mass/inertia, hydrostatic runtime, MuJoCo ellipsoid fluid forces,
waterline fluidcoef scaling, Fossen residual/added-mass wrench, and the course
buoy runtime.  Thruster commands, ArduSub, ROS, cameras, and the GUI are
deliberately excluded.

The accepted current profile has zero ambient current.  The current-response
case therefore changes only ``current_world`` in an in-memory profile copy to
exercise the exact same production current path without changing config.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles  # noqa: E402
from physics.thruster_mapping import ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER  # noqa: E402
from sim.runtime.course_buoy_runtime import CourseBuoyRuntime  # noqa: E402
from sim.runtime.hydrodynamics_runtime_setup import build_hydrodynamics_runtime_setup  # noqa: E402
from sim.runtime.model_runtime_setup import load_model_runtime_setup  # noqa: E402
from sim.runtime.parsing import to_float_array, to_float_matrix  # noqa: E402
from sim.runtime.physics_runtime_hydrostatic import build_hydrostatic_context  # noqa: E402
from sim.runtime.physics_runtime_underwater import create_underwater_wrench_runtime  # noqa: E402
from sim.runtime.pose_math import rpy_rad_from_quat_wxyz  # noqa: E402


SCENE = ROOT / "scenes" / "tank_current_scene.xml"
PROFILE_FILE = ROOT / "config" / "sim_profiles.json"
DT_S = 0.005
DECAY_DURATION_S = 4.0
TEST_CURRENT_WORLD = np.array([0.20, -0.10, 0.0], dtype=np.float64)


def _env_float(name: str, default: float) -> float:
    # Match the guarded competition runtime cadence without inheriting a
    # developer shell's tuning overrides.
    if name == "UUV_MUJOCO_TIMESTEP":
        return DT_S
    return float(default)


def _env_flag(_name: str, default: bool = False) -> bool:
    return bool(default)


def _env_get(_name: str, default: str | None = None) -> str | None:
    return default


def _silent(_message: str) -> None:
    return None


def _resolved_profile(current_world: np.ndarray) -> dict:
    profiles, warning = load_sim_profiles(PROFILE_FILE)
    if warning:
        raise AssertionError(warning)
    profile = deepcopy(build_sim_profile(profiles, "current"))
    profile["current_world"] = np.asarray(current_world, dtype=np.float64).tolist()
    return profile


def _actuator_ids(mujoco, model) -> dict[str, int]:
    return {
        str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)): actuator_id
        for actuator_id in range(int(model.nu))
    }


def _build_plant(current_world: np.ndarray):
    import mujoco

    profile = _resolved_profile(current_world)
    args = SimpleNamespace(
        scene=str(SCENE),
        fluid_model="current",
        thruster_loop_hz=80.0,
    )
    model_setup = load_model_runtime_setup(
        args=args,
        mujoco_module=mujoco,
        sim_profile=profile,
        run_mode="closed_loop",
        env_float=_env_float,
        env_flag=_env_flag,
        env_get=_env_get,
        to_float_array=to_float_array,
        to_float_matrix=to_float_matrix,
    )
    model = model_setup.model
    data = model_setup.data
    base_state = model_setup.base_state
    actuator_ids = _actuator_ids(mujoco, model)
    perf_cfg = {
        "active": False,
        "direct": False,
        "force": np.zeros(0, dtype=np.float64),
    }
    hydrostatic_context = build_hydrostatic_context(
        mujoco_module=mujoco,
        np_module=np,
        model=model,
        data=data,
        sim_profile=profile,
        perf_cfg=perf_cfg,
        scene_fluid_density=model_setup.scene_fluid_density,
        base_id=base_state.base_id,
        world_qpos_adr=base_state.world_qpos_adr,
        world_qvel_adr=base_state.world_qvel_adr,
        real_start_required=False,
        actuator_ids=actuator_ids,
        horizontal_order=list(ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER),
        env_float=_env_float,
        env_flag=_env_flag,
        log=_silent,
    )
    hydrodynamics = build_hydrodynamics_runtime_setup(
        args=args,
        model=model,
        data=data,
        mujoco_module=mujoco,
        sim_profile=profile,
        hydro_cfg=hydrostatic_context.hydro_cfg,
        use_custom_hydrodynamics=False,
        active_body_components=hydrostatic_context.hydrostatic_runtime.active_body_components,
        active_buoyancy_points=hydrostatic_context.hydrostatic_runtime.active_buoyancy_points,
        fluidcoef_dynamic_setup=model_setup.fluidcoef_dynamic_setup,
        fluid_geom_names=model_setup.fluid_geom_names,
        neutral_volume=hydrostatic_context.neutral_volume,
        vehicle_mass=hydrostatic_context.vehicle_mass,
        rho=hydrostatic_context.rho,
        water_surface_z=base_state.water_surface_z,
        thruster_air_force_scale=0.0,
        thruster_immersion_half_height_m=0.045,
        env_float=_env_float,
        env_flag=_env_flag,
        to_float_array=to_float_array,
        log=_silent,
    )
    dummy_thruster = SimpleNamespace(
        last_reaction_torque_world=np.zeros(3, dtype=np.float64),
    )
    underwater, body_velocity_local = create_underwater_wrench_runtime(
        mujoco_module=mujoco,
        model=model,
        data=data,
        base_id=base_state.base_id,
        world_qpos_adr=base_state.world_qpos_adr,
        world_qvel_adr=base_state.world_qvel_adr,
        water_surface_z=base_state.water_surface_z,
        use_custom_hydrodynamics=False,
        hydrostatic_context=hydrostatic_context,
        hydro_runtime=hydrodynamics,
        initial_depth_hold={},
        thruster_actuator_runtime=dummy_thruster,
        base_origin_world=base_state.base_origin_world,
    )
    course_buoys = CourseBuoyRuntime.from_model(
        mujoco_module=mujoco,
        model=model,
        data=data,
        water_surface_z=base_state.water_surface_z,
        water_current_world=hydrodynamics.water_current_world,
        env_float=_env_float,
        env_flag=_env_flag,
        log=_silent,
    )
    return SimpleNamespace(
        mujoco=mujoco,
        profile=profile,
        model=model,
        data=data,
        base_state=base_state,
        hydrostatic_context=hydrostatic_context,
        hydrodynamics=hydrodynamics,
        underwater=underwater,
        course_buoys=course_buoys,
        body_velocity_local=body_velocity_local,
    )


def _set_vehicle_state(plant, velocity_world: np.ndarray) -> None:
    state = plant.base_state
    qpos_adr = int(state.world_qpos_adr)
    qvel_adr = int(state.world_qvel_adr)
    plant.data.qpos[qpos_adr : qpos_adr + 3] = [0.0, -10.0, -2.0]
    plant.data.qpos[qpos_adr + 3 : qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
    plant.data.qvel[qvel_adr : qvel_adr + 6] = np.asarray(velocity_world, dtype=np.float64)
    plant.mujoco.mj_forward(plant.model, plant.data)
    plant.underwater.prev_rel_nu_valid = False
    plant.underwater.prev_rel_sample_time_s = float("nan")


def _base_kinetic_energy(plant) -> float:
    model = plant.model
    data = plant.data
    dof_adr = int(plant.base_state.world_qvel_adr)
    full_mass = np.empty((int(model.nv), int(model.nv)), dtype=np.float64)
    # Older MuJoCo Python bindings accepted the packed ``qM`` array; current
    # bindings accept MjData directly. Support both without weakening the check.
    if hasattr(data, "qM"):
        plant.mujoco.mj_fullM(model, full_mass, data.qM)
    else:
        plant.mujoco.mj_fullM(model, data, full_mass)
    base_mass = full_mass[dof_adr : dof_adr + 6, dof_adr : dof_adr + 6]
    base_velocity = np.asarray(data.qvel[dof_adr : dof_adr + 6], dtype=np.float64)
    kinetic = 0.5 * float(base_velocity @ base_mass @ base_velocity)

    # The production residual added-mass matrix is a real part of the plant but
    # is applied as an external acceleration-dependent wrench, so it is not in
    # MuJoCo's qM and must be included explicitly in the energy bookkeeping.
    lin_body, ang_body = plant.body_velocity_local()
    relative_body = np.concatenate(
        (
            lin_body - plant.data.xmat[plant.base_state.base_id].reshape(3, 3).T @ plant.hydrodynamics.water_current_world,
            ang_body,
        )
    )
    added_mass = np.asarray(plant.hydrodynamics.fossen_residual_added_mass_matrix, dtype=np.float64)
    kinetic += 0.5 * float(relative_body @ added_mass @ relative_body)
    return kinetic


def _hydrostatic_potential(plant) -> float:
    hs = plant.underwater.hydrostatic
    data = plant.data
    base_id = int(plant.base_state.base_id)
    com_z = float(data.subtree_com[base_id, 2])
    buoyancy_z = float(plant.underwater.last_buoy_force[2])
    buoyancy_point_z = float(plant.underwater.last_buoy_point[2])
    effective_buoyancy_z = com_z + float(hs.cob_torque_scale) * (buoyancy_point_z - com_z)
    gravity_potential = plant.hydrostatic_context.vehicle_mass * plant.hydrostatic_context.gravity * com_z
    buoyancy_potential = -buoyancy_z * effective_buoyancy_z

    qpos_adr = int(plant.base_state.world_qpos_adr)
    roll, pitch, _yaw = rpy_rad_from_quat_wxyz(data.qpos[qpos_adr + 3 : qpos_adr + 7])
    restoring_potential = 0.5 * float(hs.hydrostatic_restoring_roll_stiffness) * roll * roll
    restoring_potential += 0.5 * float(hs.hydrostatic_restoring_pitch_stiffness) * pitch * pitch
    return float(gravity_potential + buoyancy_potential + restoring_potential)


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def _check_six_dof_free_decay() -> dict[str, float]:
    plant = _build_plant(np.zeros(3, dtype=np.float64))
    initial_velocity = np.array([0.30, -0.25, 0.18, 0.35, -0.28, 0.40], dtype=np.float64)
    _set_vehicle_state(plant, initial_velocity)
    dt = float(plant.model.opt.timestep)
    if abs(dt - DT_S) > 1.0e-12:
        raise AssertionError(f"competition timestep mismatch: {dt:.9f}s")

    linear_speeds: list[float] = []
    angular_speeds: list[float] = []
    energies: list[float] = []
    initial_potential: float | None = None
    steps = int(round(DECAY_DURATION_S / dt))
    for step in range(steps + 1):
        plant.underwater.apply(dt)
        plant.course_buoys.apply(dt)
        lin_body, ang_body = plant.body_velocity_local()
        if not np.all(np.isfinite(lin_body)) or not np.all(np.isfinite(ang_body)):
            raise AssertionError(f"non-finite 6DOF velocity at step {step}")
        linear_speeds.append(float(np.linalg.norm(lin_body)))
        angular_speeds.append(float(np.linalg.norm(ang_body)))
        if step % 5 == 0:
            potential = _hydrostatic_potential(plant)
            if initial_potential is None:
                initial_potential = potential
            energies.append(_base_kinetic_energy(plant) + potential - initial_potential)
        if step < steps:
            plant.mujoco.mj_step(plant.model, plant.data)

    linear = np.asarray(linear_speeds, dtype=np.float64)
    angular = np.asarray(angular_speeds, dtype=np.float64)
    energy = np.asarray(energies, dtype=np.float64)
    if not np.all(np.isfinite(energy)):
        raise AssertionError("non-finite free-decay energy")
    window = max(20, int(0.20 * linear.size))
    linear_early = _rms(linear[:window])
    linear_late = _rms(linear[-window:])
    angular_early = _rms(angular[:window])
    angular_late = _rms(angular[-window:])
    energy_initial = float(energy[0])
    energy_peak = float(np.max(energy))
    energy_final = float(energy[-1])

    if not linear_late < linear_early:
        raise AssertionError(f"linear motion did not damp: early={linear_early:.6f}, late={linear_late:.6f}")
    if not angular_late < angular_early:
        raise AssertionError(f"angular motion did not damp: early={angular_early:.6f}, late={angular_late:.6f}")
    energy_growth_limit = max(0.01, 0.05 * max(energy_initial, 0.01))
    if energy_peak > energy_initial + energy_growth_limit:
        raise AssertionError(
            "free-decay energy grew beyond 5% numerical allowance: "
            f"initial={energy_initial:.6f}J peak={energy_peak:.6f}J"
        )
    if not energy_final < energy_initial:
        raise AssertionError(f"free-decay energy did not fall: initial={energy_initial:.6f}J final={energy_final:.6f}J")
    return {
        "linear_rms_early_mps": linear_early,
        "linear_rms_late_mps": linear_late,
        "angular_rms_early_radps": angular_early,
        "angular_rms_late_radps": angular_late,
        "energy_initial_j": energy_initial,
        "energy_peak_j": energy_peak,
        "energy_final_j": energy_final,
    }


def _horizontal_acceleration(plant, velocity_world: np.ndarray) -> np.ndarray:
    _set_vehicle_state(plant, velocity_world)
    plant.underwater.apply(float(plant.model.opt.timestep))
    # Re-run forward dynamics after the production callback populated
    # xfrc_applied and adjusted immersed fluid coefficients.
    plant.mujoco.mj_forward(plant.model, plant.data)
    dof_adr = int(plant.base_state.world_qvel_adr)
    acceleration = np.asarray(plant.data.qacc[dof_adr : dof_adr + 2], dtype=np.float64).copy()
    if not np.all(np.isfinite(acceleration)):
        raise AssertionError(f"non-finite current response: {acceleration}")
    return acceleration


def _check_configured_current_response() -> dict[str, float]:
    plant = _build_plant(TEST_CURRENT_WORLD)
    np.testing.assert_allclose(plant.hydrodynamics.water_current_world, TEST_CURRENT_WORLD, atol=1.0e-12)
    np.testing.assert_allclose(plant.model.opt.wind, TEST_CURRENT_WORLD, atol=1.0e-12)
    np.testing.assert_allclose(plant.course_buoys.water_current_world, TEST_CURRENT_WORLD, atol=1.0e-12)

    stationary_accel = _horizontal_acceleration(plant, np.zeros(6, dtype=np.float64))
    # Rebuild to guarantee no added-mass history from the stationary sample can
    # leak into the Galilean co-moving sample.
    comoving = _build_plant(TEST_CURRENT_WORLD)
    comoving_velocity = np.zeros(6, dtype=np.float64)
    comoving_velocity[:3] = TEST_CURRENT_WORLD
    comoving_accel = _horizontal_acceleration(comoving, comoving_velocity)

    current_xy = TEST_CURRENT_WORLD[:2]
    along_current = float(np.dot(stationary_accel, current_xy) / np.linalg.norm(current_xy))
    stationary_norm = float(np.linalg.norm(stationary_accel))
    comoving_norm = float(np.linalg.norm(comoving_accel))
    if along_current <= 1.0e-4:
        raise AssertionError(
            "stationary vehicle did not accelerate with configured current: "
            f"current={current_xy} acceleration={stationary_accel}"
        )
    if np.any(stationary_accel * current_xy <= 0.0):
        raise AssertionError(
            "configured-current component direction mismatch: "
            f"current={current_xy} acceleration={stationary_accel}"
        )
    if comoving_norm > max(1.0e-5, 0.02 * stationary_norm):
        raise AssertionError(
            "Galilean co-moving acceleration is too large: "
            f"stationary={stationary_accel} comoving={comoving_accel}"
        )
    return {
        "current_x_mps": float(TEST_CURRENT_WORLD[0]),
        "current_y_mps": float(TEST_CURRENT_WORLD[1]),
        "stationary_ax_mps2": float(stationary_accel[0]),
        "stationary_ay_mps2": float(stationary_accel[1]),
        "comoving_accel_norm_mps2": comoving_norm,
    }


def main() -> int:
    decay = _check_six_dof_free_decay()
    current = _check_configured_current_response()
    metrics = {**decay, **current}
    rendered = " ".join(f"{key}={value:.6g}" for key, value in metrics.items())
    print(f"vehicle_fluid_free_decay=PASS {rendered}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
