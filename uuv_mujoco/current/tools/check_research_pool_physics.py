#!/usr/bin/env python3
"""Actual-MuJoCo smoke for the research-pool hydrodynamic extensions."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles  # noqa: E402
from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)
from sim.physics.fluid_contract import configure_fluid_model_contract  # noqa: E402
from sim.runtime.hydrodynamics_runtime_setup import build_hydrodynamics_runtime_setup  # noqa: E402
from sim.runtime.model_runtime_setup import load_model_runtime_setup  # noqa: E402
from sim.runtime.parsing import to_float_array, to_float_matrix  # noqa: E402
from sim.runtime.physics_runtime_hydrostatic import build_hydrostatic_context  # noqa: E402
from sim.runtime.physics_runtime_thruster_actuator import (  # noqa: E402
    create_thruster_actuator_runtime,
)
from sim.runtime.physics_runtime_underwater import create_underwater_wrench_runtime  # noqa: E402
from sim.runtime.thruster_param_runtime import ThrusterParameterRuntime  # noqa: E402


SCENE = ROOT / "scenes" / "research_pool_slam_scene.xml"
PROFILE = ROOT / "config" / "sim_profiles.json"
DT_S = 0.005


def _env_float(name: str, default: float) -> float:
    if name == "UUV_MUJOCO_TIMESTEP":
        return DT_S
    return float(default)


def _env_flag(name: str, default: bool = False) -> bool:
    if name == "UUV_MUJOCO_MODEL_CACHE":
        return False
    return bool(default)


def _env_get(_name: str, default: str | None = None) -> str | None:
    return default


def _silent(_message: str) -> None:
    return None


def _build_runtime(
    mujoco,
    *,
    profile_name: str = "research_pool",
    fluid_model: str = "current",
    use_custom_hydrodynamics: bool = False,
):
    profiles, warning = load_sim_profiles(PROFILE)
    if warning:
        raise AssertionError(warning)
    profile = build_sim_profile(profiles, profile_name)
    args = SimpleNamespace(
        scene=str(SCENE),
        fluid_model=fluid_model,
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
    state = model_setup.base_state
    contract_custom = configure_fluid_model_contract(
        model=model,
        fluid_model=fluid_model,
        scene_path=str(SCENE),
        scene_fluid_density=model_setup.scene_fluid_density,
        scene_fluid_viscosity=model_setup.scene_fluid_viscosity,
    )
    if bool(contract_custom) != bool(use_custom_hydrodynamics):
        raise AssertionError("test helper fluid ownership does not match requested runtime")
    actuator_ids = {
        str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)): index
        for index in range(int(model.nu))
    }
    hydrostatic = build_hydrostatic_context(
        mujoco_module=mujoco,
        np_module=np,
        model=model,
        data=data,
        sim_profile=profile,
        perf_cfg={"active": False, "direct": False, "force": np.zeros(0)},
        scene_fluid_density=model_setup.scene_fluid_density,
        base_id=state.base_id,
        world_qpos_adr=state.world_qpos_adr,
        world_qvel_adr=state.world_qvel_adr,
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
        hydro_cfg=hydrostatic.hydro_cfg,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        active_body_components=hydrostatic.hydrostatic_runtime.active_body_components,
        active_buoyancy_points=hydrostatic.hydrostatic_runtime.active_buoyancy_points,
        fluidcoef_dynamic_setup=model_setup.fluidcoef_dynamic_setup,
        fluid_geom_names=model_setup.fluid_geom_names,
        neutral_volume=hydrostatic.neutral_volume,
        vehicle_mass=hydrostatic.vehicle_mass,
        rho=hydrostatic.rho,
        water_surface_z=state.water_surface_z,
        thruster_air_force_scale=0.0,
        thruster_immersion_half_height_m=0.045,
        env_float=_env_float,
        env_flag=_env_flag,
        to_float_array=to_float_array,
        log=_silent,
    )
    vertical_thrusters = list(PHYSICAL_VERTICAL_THRUSTERS)
    horizontal_thrusters = list(PHYSICAL_YAW_THRUSTERS)
    all_thruster_names = vertical_thrusters + horizontal_thrusters
    thruster_params = ThrusterParameterRuntime.create(all_thruster_names)
    thruster_params.load(
        path=ROOT / "config" / "thruster_params.json",
        thruster_names=all_thruster_names,
        sim_profile=profile,
        vertical_thrusters=vertical_thrusters,
        horizontal_thrusters=horizontal_thrusters,
        env_get=_env_get,
        log=_silent,
    )
    thruster_actuator = create_thruster_actuator_runtime(
        model=model,
        data=data,
        mujoco_module=mujoco,
        actuator_ids=actuator_ids,
        ctrlrange=model.actuator_ctrlrange.copy(),
        all_thruster_names=all_thruster_names,
        thruster_global=thruster_params.global_params,
        thruster_scale=thruster_params.scale,
        thruster_direct_scale=thruster_params.direct_scale,
        thruster_reverse_asymmetry=thruster_params.reverse_asymmetry,
        thruster_tau_up=thruster_params.tau_up,
        thruster_tau_down=thruster_params.tau_down,
        perf_cfg={"active": False, "direct": False, "force": np.zeros(0)},
        thruster_force_max=hydrodynamics.thruster_force_max,
        water_surface_z=state.water_surface_z,
        thruster_air_force_scale=0.0,
        thruster_immersion_half_height_m=0.045,
        buoyancy_model=hydrodynamics.buoyancy_model,
        yaw_torque_scale=hydrodynamics.yaw_torque_scale,
        yaw_torque_thruster_scales=hydrodynamics.yaw_torque_thruster_scales,
        yaw_thrusters=horizontal_thrusters,
        spin_gain=hydrodynamics.spin_gain,
    )
    thruster_actuator.set_current_velocity_sampler(
        hydrodynamics.water_environment_runtime.velocity_world
    )
    thruster_actuator.set_surface_height_sampler(
        hydrodynamics.water_environment_runtime.surface_height_world_m
    )
    underwater, _body_velocity_local = create_underwater_wrench_runtime(
        mujoco_module=mujoco,
        model=model,
        data=data,
        base_id=state.base_id,
        world_qpos_adr=state.world_qpos_adr,
        world_qvel_adr=state.world_qvel_adr,
        water_surface_z=state.water_surface_z,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        hydrostatic_context=hydrostatic,
        hydro_runtime=hydrodynamics,
        initial_depth_hold={},
        thruster_actuator_runtime=thruster_actuator,
        base_origin_world=state.base_origin_world,
    )
    return SimpleNamespace(
        model=model,
        data=data,
        state=state,
        hydrodynamics=hydrodynamics,
        underwater=underwater,
        thruster_actuator=thruster_actuator,
        fluid_geom_ids=np.asarray(model_setup.fluid_geom_ids, dtype=np.int32),
    )


def main() -> int:
    import mujoco

    runtime = _build_runtime(mujoco)
    model = runtime.model
    data = runtime.data
    hyd = runtime.hydrodynamics
    underwater = runtime.underwater
    if not hyd.current_field_runtime.field.active:
        raise AssertionError("research current field is not active in the production runtime")
    if not hyd.state_coefficient_scaler.active:
        raise AssertionError("research state coefficient scaling is not active")
    if runtime.fluid_geom_ids.size != 3:
        raise AssertionError(f"expected three vehicle fluid proxies, got {runtime.fluid_geom_ids.size}")

    initial_position = data.xipos[int(runtime.state.base_id)].copy()
    expected_current = hyd.current_field_runtime.field.velocity_world(initial_position, float(data.time))
    underwater.apply(DT_S)
    np.testing.assert_allclose(hyd.water_current_world, expected_current, atol=1.0e-12)
    np.testing.assert_allclose(model.opt.wind, expected_current, atol=1.0e-12)
    if underwater.last_hydrodynamic_state_scales is None:
        raise AssertionError("state coefficient scale did not reach the underwater wrench runtime")

    first_coefficients = model.geom_fluid[runtime.fluid_geom_ids, 1:6].copy()
    underwater.apply(DT_S)
    np.testing.assert_allclose(
        model.geom_fluid[runtime.fluid_geom_ids, 1:6],
        first_coefficients,
        atol=1.0e-12,
        err_msg="state fluidcoef scale compounded at an unchanged state",
    )

    qpos = int(runtime.state.world_qpos_adr)
    data.qpos[qpos] += 8.0
    data.qpos[qpos + 1] -= 2.5
    mujoco.mj_forward(model, data)
    moved_position = data.xipos[int(runtime.state.base_id)].copy()
    underwater.prev_rel_nu_valid = False
    underwater.apply(DT_S)
    moved_current = hyd.water_current_world.copy()
    if np.allclose(moved_current, expected_current):
        raise AssertionError("spatial current did not change after moving the vehicle")
    if float(np.linalg.norm(moved_current)) > 0.08 + 1.0e-12:
        raise AssertionError(f"current exceeded profile bound: {moved_current}")

    for _index in range(25):
        underwater.apply(DT_S)
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise AssertionError("research-pool hydrodynamics produced non-finite state")

    print(
        "research_pool_physics=PASS "
        f"initial_current={np.array2string(expected_current, precision=4)} "
        f"moved_current={np.array2string(moved_current, precision=4)} "
        "steps=25 calibration=uncalibrated_pool_prior"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
