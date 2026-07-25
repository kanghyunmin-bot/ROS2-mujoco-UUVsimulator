"""Forced GUI-start simulator environment contract."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

from .sim_stack_env_forced_command import command_readiness_contract
from .sim_stack_env_forced_ekf import ekf_sensor_contract
from .sim_stack_env_forced_rc import rc_override_contract
from .sim_stack_env_flags import env_bool


def default_real_start_csv(sim_stack_dir: Path) -> Path:
    return (
        sim_stack_dir
        / "debug"
        / "controller_parity_412"
        / "real_20260401_feedback"
        / "real_controller_feedback_20hz.csv"
    )


def base_forced_gui_contract(env: Mapping[str, str], *, ekf_contract: str) -> dict[str, str]:
    return {
        **rc_override_contract(),
        **ekf_sensor_contract(env, ekf_contract=ekf_contract),
        **command_readiness_contract(env),
    }


def apply_real_start_contract(
    forced: dict[str, str],
    env: Mapping[str, str],
    *,
    run_mode: str,
    real_start_csv: Path,
) -> None:
    if run_mode == "closed_loop" and env_bool(env, "UUV_REAL_START_STATE") and real_start_csv.exists():
        forced.update(
            {
                "UUV_REAL_START_STATE": "1",
                "UUV_REAL_START_STATE_CSV": env.get("UUV_REAL_START_STATE_CSV", str(real_start_csv)),
                "UUV_REAL_START_STATE_T_S": env.get("UUV_REAL_START_STATE_T_S", "69.35"),
                "UUV_REAL_START_STATE_HOLD_UNTIL_RELEASE": env.get(
                    "UUV_REAL_START_STATE_HOLD_UNTIL_RELEASE",
                    "0",
                ),
                "UUV_REAL_START_STATE_AUTO_RELEASE": env.get(
                    "UUV_REAL_START_STATE_AUTO_RELEASE",
                    "1",
                ),
            }
        )
        if "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE" in env:
            forced["UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE"] = env[
                "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE"
            ]
    elif run_mode == "closed_loop":
        forced["UUV_REAL_START_STATE"] = "0"
        forced["UUV_YAW_TORQUE_SCALE"] = "1.0"


def apply_run_mode_contract(forced: dict[str, str], *, run_mode: str) -> None:
    if run_mode == "plant_replay":
        forced.update(
            {
                "ROS2_UUV_SITL_JSON_SERVO_FALLBACK": "0",
                "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE": "1",
            }
        )
    else:
        forced.update(
            {
                "ROS2_UUV_SITL_JSON_SERVO_FALLBACK": "1",
                "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE": "0",
            }
        )


def apply_pilot_heave_response_contract(
    forced: dict[str, str],
    env: Mapping[str, str],
    *,
    run_mode: str,
) -> None:
    if run_mode == "plant_replay":
        return
    # GUI closed-loop control should expose the controller path first. The
    # current profile carries plant-replay heave residual terms; keep those
    # opt-in for A/B by preserving explicit env overrides, but do not let them
    # make live RC heave feel delayed by default.
    defaults = {
        "UUV_HEAVE_EXTRA_DAMPING_N_PER_MPS": "0.0",
        "UUV_HYDRO_VERTICAL_LIFT_COEFF": "0.0",
        "UUV_HYDRO_YAWRATE_HEAVE_POS_COEFF": "0.0",
        "UUV_HYDRO_YAWRATE_HEAVE_NEG_COEFF": "0.0",
        "UUV_YAW_TORQUE_SCALE": "1.0",
        "UUV_YAW_TORQUE_THRUSTER_SCALES_ENABLE": "0.0",
        # Direct mode is the raw final-PWM -> measured T200 curve contract.
        # Group gains above one apply actuator authority a second time and can
        # drive the stock ArduSub rate loop into saturation.
        "UUV_HORIZONTAL_DIRECT_GAIN_SCALE": "1.0",
        "UUV_VERTICAL_DIRECT_GAIN_SCALE": "1.0",
    }
    for key, value in defaults.items():
        forced[key] = env.get(key, value)


def apply_forced_gui_contract(
    env: dict[str, str],
    *,
    run_mode: str,
    ekf_contract: str,
    sim_stack_dir: Path,
    explicit_keys: set[str],
) -> None:
    forced = base_forced_gui_contract(env, ekf_contract=ekf_contract)
    apply_real_start_contract(
        forced,
        env,
        run_mode=run_mode,
        real_start_csv=default_real_start_csv(sim_stack_dir),
    )
    apply_run_mode_contract(forced, run_mode=run_mode)
    apply_pilot_heave_response_contract(forced, env, run_mode=run_mode)
    for key, value in forced.items():
        if key not in explicit_keys:
            env[key] = value


__all__ = [
    "apply_pilot_heave_response_contract",
    "default_real_start_csv",
    "base_forced_gui_contract",
    "apply_real_start_contract",
    "apply_run_mode_contract",
    "apply_forced_gui_contract",
]
