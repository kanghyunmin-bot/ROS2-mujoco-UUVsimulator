"""Case runners for initial hold pose smoke checks."""

from __future__ import annotations

import numpy as np

from sim.runtime.initial_hold_pose import (
    apply_initial_hold_pose_or_depth,
    capture_initial_hold_pose,
    coerce_body_vector,
)

from initial_hold_pose_smoke_fixture import FakeMujoco, assert_equal, fake_hold_data, hold_depth_recorders


def check_capture_and_coerce(data) -> np.ndarray:
    pose = capture_initial_hold_pose(data=data, world_qpos_adr=1, water_surface_z=10.0, depth_m=2.5)
    expected = np.array([1.0, 2.0, 7.5, 4.0, 5.0, 6.0, 7.0])
    if not np.allclose(pose, expected):
        raise AssertionError(f"captured pose mismatch: {pose}")
    if not np.allclose(coerce_body_vector([1, 2, 3]), np.array([1.0, 2.0, 3.0])):
        raise AssertionError("coerce_body_vector mismatch")
    return pose


def check_captured_pose_priority(data, pose: np.ndarray) -> tuple[list[tuple[str, float, bool]], FakeMujoco]:
    calls, set_bar30_depth, set_base_depth = hold_depth_recorders()
    mujoco = FakeMujoco()
    apply_initial_hold_pose_or_depth(
        active=True,
        depth_m=1.0,
        bar30_depth_m=2.0,
        pose_qpos=pose,
        data=data,
        mujoco=mujoco,
        model=object(),
        world_qpos_adr=1,
        world_qvel_adr=2,
        set_bar30_depth=set_bar30_depth,
        set_base_depth=set_base_depth,
    )
    assert_equal(mujoco.forward_calls, 1, "captured pose forward")
    assert_equal(calls, [], "captured pose suppresses depth fallback")
    if not np.allclose(data.qvel[2:8], 0.0) or not np.allclose(data.qacc[2:8], 0.0):
        raise AssertionError("captured pose should zero qvel/qacc")
    return calls, mujoco


def check_depth_fallback_priority(data, calls: list[tuple[str, float, bool]], mujoco: FakeMujoco) -> None:
    _, set_bar30_depth, set_base_depth = hold_depth_recorders()
    apply_initial_hold_pose_or_depth(
        active=True,
        depth_m=1.0,
        bar30_depth_m=2.0,
        pose_qpos=np.array([np.nan] * 7),
        data=data,
        mujoco=mujoco,
        model=object(),
        world_qpos_adr=1,
        world_qvel_adr=2,
        set_bar30_depth=lambda value, *, reset_velocity: calls.append(("bar30", float(value), bool(reset_velocity))),
        set_base_depth=set_base_depth,
    )
    assert_equal(calls[-1], ("bar30", 2.0, True), "bar30 fallback priority")
    apply_initial_hold_pose_or_depth(
        active=True,
        depth_m=1.0,
        bar30_depth_m=None,
        pose_qpos=None,
        data=data,
        mujoco=mujoco,
        model=object(),
        world_qpos_adr=1,
        world_qvel_adr=2,
        set_bar30_depth=set_bar30_depth,
        set_base_depth=lambda value, *, reset_velocity: calls.append(("base", float(value), bool(reset_velocity))),
    )
    assert_equal(calls[-1], ("base", 1.0, True), "base fallback")


def run_initial_hold_pose_smoke() -> None:
    data = fake_hold_data()
    pose = check_capture_and_coerce(data)
    calls, mujoco = check_captured_pose_priority(data, pose)
    check_depth_fallback_priority(data, calls, mujoco)


__all__ = ["run_initial_hold_pose_smoke"]
