"""Smoke cases for residual hydrodynamics runtime builders."""

from __future__ import annotations

from sim.physics.fossen_residual_builders import build_fossen_residual_runtime, build_residual_hydro_runtime


def env_float(_name: str, default: float) -> float:
    return float(default)


def env_flag(_name: str, default: bool) -> bool:
    return bool(default)


def assert_true(value: bool, label: str) -> None:
    if not value:
        raise AssertionError(label)


def assert_false(value: bool, label: str) -> None:
    if value:
        raise AssertionError(label)


def check_residual_hydro_runtime() -> None:
    residual_profile = {
        "hydro_residual_wrench": {
            "active": True,
            "linear": {"y_v": 1.25},
        }
    }
    residual = build_residual_hydro_runtime(
        residual_profile,
        use_custom_hydrodynamics=False,
        env_float=env_float,
    )
    assert_true(residual.active, "residual hydro active with nonzero coefficient")
    assert_false(
        build_residual_hydro_runtime(
            residual_profile,
            use_custom_hydrodynamics=True,
            env_float=env_float,
        ).active,
        "custom hydrodynamics disables residual hydro",
    )


def check_fossen_residual_runtime() -> None:
    fossen_profile = {
        "fossen_residual_hydro": {
            "active": True,
            "linear": {"y_v": 2.0},
            "added_mass": {"active": True, "x_u": 1.0, "y_r_n_v": 0.5},
        }
    }
    fossen = build_fossen_residual_runtime(
        fossen_profile,
        use_custom_hydrodynamics=False,
        env_float=env_float,
        env_flag=env_flag,
    )
    assert_true(fossen.active, "fossen residual active")
    assert_true(fossen.added_mass_active, "added mass active")
    if abs(float(fossen.added_mass_matrix[0, 0]) - 1.0) > 1.0e-9:
        raise AssertionError("added mass diagonal not preserved")
    if abs(float(fossen.added_mass_matrix[1, 5]) - 0.5) > 1.0e-9:
        raise AssertionError("added mass coupling not preserved")

    custom = build_fossen_residual_runtime(
        fossen_profile,
        use_custom_hydrodynamics=True,
        env_float=env_float,
        env_flag=env_flag,
    )
    assert_false(custom.active, "custom hydrodynamics disables Fossen residual")
    assert_false(custom.added_mass_active, "custom hydrodynamics disables added mass")


__all__ = ["check_fossen_residual_runtime", "check_residual_hydro_runtime"]
