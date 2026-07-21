#!/usr/bin/env python3
"""Run deterministic buoy-physics acceptance stages in the canonical viewer.

This is an evidence/visualization harness, not another buoy simulator.  It
loads ``tank_current_scene.xml`` and drives the production
``CourseBuoyRuntime`` through the same fixtures used by
``check_buoy_collector_capture.py``.  The default mode opens a passive MuJoCo
viewer; ``--headless`` is available for CI and contract checks.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import traceback
from typing import Any, Iterable

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parent
ROOT = TOOLS_DIR.parent
GENERATED = ROOT / "generated"
SCENE = ROOT / "scenes" / "tank_current_scene.xml"

# Import the canonical test fixture instead of maintaining a second set of
# model/runtime helpers in this viewer-only harness.
sys.path.insert(0, str(TOOLS_DIR))
import check_buoy_collector_capture as fixture  # noqa: E402


SCHEMA = "uuv.buoy_viewer_acceptance.v1"


def _vec(values: Any) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).reshape(-1)]


def _finite_nonnegative(value: str) -> float:
    parsed = float(value)
    if not np.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def _bounded_positive(value: str) -> float:
    parsed = float(value)
    if not np.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


class EvidenceWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", encoding="utf-8", buffering=1)
        self._sequence = 0

    def close(self) -> None:
        self._stream.close()

    def emit(self, event: str, **payload: Any) -> dict[str, Any]:
        self._sequence += 1
        record = {
            "schema": SCHEMA,
            "sequence": self._sequence,
            "event": event,
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            **payload,
        }
        self._stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        return record


@dataclass
class ContactTrace:
    """Bounded aggregate of contact-pair evidence for one acceptance stage."""

    tokens: tuple[str, ...] = ()
    max_pairs: int = 48
    pairs: dict[tuple[str, str], dict[str, Any]] = field(default_factory=dict)

    def observe(self, mujoco: Any, model: Any, data: Any) -> None:
        for index in range(int(data.ncon)):
            contact = data.contact[index]
            names = []
            for geom_id in (int(contact.geom1), int(contact.geom2)):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                names.append(name or f"geom:{geom_id}")
            if self.tokens and not any(token in name for token in self.tokens for name in names):
                continue
            key = tuple(sorted(names))
            if key not in self.pairs and len(self.pairs) >= self.max_pairs:
                continue
            force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, index, force)
            force_norm_n = float(np.linalg.norm(force[:3]))
            row = self.pairs.setdefault(
                key,
                {
                    "geom_pair": list(key),
                    "samples": 0,
                    "min_distance_m": float(contact.dist),
                    "max_force_n": 0.0,
                },
            )
            row["samples"] = int(row["samples"]) + 1
            row["min_distance_m"] = min(float(row["min_distance_m"]), float(contact.dist))
            row["max_force_n"] = max(float(row["max_force_n"]), force_norm_n)

    def contains(self, left: str, right: str) -> bool:
        return tuple(sorted((left, right))) in self.pairs

    def peak_force(self, left: str, right: str) -> float:
        row = self.pairs.get(tuple(sorted((left, right))))
        return 0.0 if row is None else float(row["max_force_n"])

    def rows(self) -> list[dict[str, Any]]:
        return [self.pairs[key] for key in sorted(self.pairs)]


class ViewerDriver:
    """Throttle passive-viewer sync without changing physics cadence."""

    def __init__(
        self,
        *,
        mujoco: Any,
        model: Any,
        data: Any,
        viewer: Any | None,
        render_hz: float,
        checkpoint_hold_s: float,
    ) -> None:
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.viewer = viewer
        self.render_hz = min(60.0, max(5.0, float(render_hz)))
        self.checkpoint_hold_s = min(10.0, max(0.0, float(checkpoint_hold_s)))
        self._speed = 1.0
        self._next_sync_time_s = float(data.time)
        self._focus: np.ndarray | None = None
        self._distance = 2.5
        self._azimuth = 135.0
        self._elevation = -18.0

    def stage(self, *, speed: float) -> None:
        self._speed = min(50.0, max(0.25, float(speed)))
        self._next_sync_time_s = float(self.data.time)

    def focus(
        self,
        xyz: Iterable[float],
        *,
        distance: float = 2.5,
        azimuth: float = 135.0,
        elevation: float = -18.0,
    ) -> None:
        self._focus = np.asarray(tuple(xyz), dtype=np.float64).reshape(3)
        self._distance = float(distance)
        self._azimuth = float(azimuth)
        self._elevation = float(elevation)

    def _apply_camera(self) -> None:
        if self.viewer is None or self._focus is None:
            return
        cam = self.viewer.cam
        cam.type = int(self.mujoco.mjtCamera.mjCAMERA_FREE)
        cam.fixedcamid = -1
        cam.trackbodyid = -1
        cam.lookat[:] = self._focus
        cam.distance = self._distance
        cam.azimuth = self._azimuth
        cam.elevation = self._elevation

    def sync(self, *, force: bool = False) -> None:
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            raise RuntimeError("MuJoCo viewer was closed before acceptance completed")
        now_s = float(self.data.time)
        if not force and now_s + 1.0e-12 < self._next_sync_time_s:
            return
        self._apply_camera()
        self.viewer.sync(state_only=True)
        self._next_sync_time_s = now_s + self._speed / self.render_hz
        # One displayed frame per wall-clock render period makes the selected
        # stage speed approximately `_speed` times real time.
        if not force:
            time.sleep(1.0 / self.render_hz)

    def hold(self, seconds: float | None = None) -> None:
        if self.viewer is None:
            return
        duration = self.checkpoint_hold_s if seconds is None else max(0.0, float(seconds))
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if not self.viewer.is_running():
                raise RuntimeError("MuJoCo viewer was closed before acceptance completed")
            self._apply_camera()
            self.viewer.sync(state_only=True)
            time.sleep(min(1.0 / self.render_hz, max(0.0, deadline - time.monotonic())))

    def keep_open(self) -> None:
        if self.viewer is None:
            return
        while self.viewer.is_running():
            self._apply_camera()
            self.viewer.sync(state_only=True)
            time.sleep(1.0 / self.render_hz)


def _scene_sha256() -> str:
    digest = hashlib.sha256()
    with SCENE.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _capture_state(runtime: Any, buoy: Any) -> str:
    return str(runtime.status_by_name()[buoy.name]["capture_state"])


def _buoy_evidence(runtime: Any, buoy: Any, *, base_id: int) -> dict[str, Any]:
    data = runtime.data
    center = runtime._buoy_center_world(buoy)
    local = fixture.base_local_from_world(data, base_id, center)
    return {
        "name": buoy.name,
        "center_world_xyz_m": _vec(center),
        "center_base_local_xyz_m": _vec(local),
        "free_qpos": _vec(data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7]),
        "free_qvel": _vec(data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6]),
        "detached": bool(buoy.detached),
        "surface_on_waterline": bool(buoy.surface_on_waterline),
        "capture_state": _capture_state(runtime, buoy),
        "netting": bool(buoy.netting),
        "netted": bool(buoy.netted),
        "net_reverse_released": bool(buoy.net_reverse_released),
        "net_score_released": bool(buoy.net_score_released),
        "net_slot_index": int(buoy.net_slot_index),
        "magnet_eq_active": bool(buoy.eq_id >= 0 and data.eq_active[buoy.eq_id]),
        "collector_eq_active": bool(
            buoy.collector_eq_id >= 0 and data.eq_active[buoy.collector_eq_id]
        ),
    }


def _checkpoint(
    writer: EvidenceWriter,
    *,
    name: str,
    runtime: Any,
    base_id: int,
    buoys: Iterable[Any],
    trace: ContactTrace | None = None,
    metrics: dict[str, Any] | None = None,
) -> None:
    writer.emit(
        "checkpoint",
        checkpoint=name,
        passed=True,
        sim_time_s=float(runtime.data.time),
        vehicle_world_xyz_m=_vec(runtime.data.xpos[base_id]),
        buoys=[_buoy_evidence(runtime, buoy, base_id=base_id) for buoy in buoys],
        contacts=[] if trace is None else trace.rows(),
        metrics={} if metrics is None else metrics,
    )
    print(f"[acceptance] {name}: PASS", flush=True)


def _set_vehicle_pose(
    mujoco: Any,
    model: Any,
    data: Any,
    *,
    qposadr: int,
    dofadr: int,
    xyz: Iterable[float],
) -> None:
    data.qpos[qposadr : qposadr + 3] = np.asarray(tuple(xyz), dtype=np.float64)
    data.qpos[qposadr + 3 : qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[dofadr : dofadr + 6] = 0.0
    mujoco.mj_forward(model, data)


def _run_rake_stages(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    runtime: Any,
    driver: ViewerDriver,
    writer: EvidenceWriter,
    base_id: int,
    base_qposadr: int,
    base_dofadr: int,
    ascent_speed: float,
) -> None:
    dt = float(model.opt.timestep)
    fixture.require(
        runtime.contact_release_hold_s <= 0.0,
        f"canonical immediate-release contract is disabled: hold={runtime.contact_release_hold_s}",
    )

    def strike(buoy_name: str, *, verify_gap: bool) -> dict[str, float]:
        buoy = next(item for item in runtime.buoys if item.name == buoy_name)
        buoy_position = runtime._buoy_center_world(buoy).copy()
        gap_y = -0.1000

        if verify_gap:
            trace = ContactTrace(tokens=(buoy_name, "rake"))
            gap_position = buoy_position + np.array([-0.390, -gap_y, -0.070])
            driver.stage(speed=1.0)
            driver.focus(buoy_position, distance=1.2, azimuth=90.0, elevation=-8.0)
            for _ in range(30):
                _set_vehicle_pose(
                    mujoco,
                    model,
                    data,
                    qposadr=base_qposadr,
                    dofadr=base_dofadr,
                    xyz=gap_position,
                )
                trace.observe(mujoco, model, data)
                runtime.apply(dt)
                mujoco.mj_step(model, data)
                driver.sync()
            fixture.require(
                not buoy.detached,
                f"{buoy_name} released while its PVC passed through the tine gap",
            )
            fixture.require(
                buoy.eq_id >= 0 and bool(data.eq_active[buoy.eq_id]),
                f"{buoy_name} magnet weld dropped in the tine gap",
            )
            _checkpoint(
                writer,
                name="rake_tine_gap_magnet_held",
                runtime=runtime,
                base_id=base_id,
                buoys=(buoy,),
                trace=trace,
                metrics={"fixture_vehicle_qpos_only": True, "gap_y_m": gap_y},
            )
            driver.sync(force=True)
            driver.hold()

        trace = ContactTrace(tokens=(buoy_name, "rake"))
        root_position = buoy_position + np.array([-0.340, -gap_y, -0.070])
        push_speed_mps = 0.35 if dt >= 0.006 else 0.45
        push_steps = int(np.ceil(0.65 / dt))
        contact_hold_s = 0.0
        contact_peak_n = 0.0
        release_hold_s = -1.0
        release_peak_n = -1.0
        qpos_jump = -1.0
        qvel_jump = -1.0
        release_z = float(buoy_position[2])

        driver.stage(speed=1.0)
        driver.focus(buoy_position, distance=1.2, azimuth=90.0, elevation=-8.0)
        for step in range(push_steps):
            _set_vehicle_pose(
                mujoco,
                model,
                data,
                qposadr=base_qposadr,
                dofadr=base_dofadr,
                xyz=root_position + np.array([push_speed_mps * dt * step, 0.0, 0.0]),
            )
            trace.observe(mujoco, model, data)
            contacted = buoy.body_id in runtime._release_probe_contacted_buoy_body_ids()
            if contacted:
                contact_hold_s += dt
                contact_peak_n = max(
                    contact_peak_n,
                    runtime._contact_force_norm(
                        buoy,
                        vehicle_geom_ids=runtime.vehicle_release_probe_geom_ids,
                    ),
                )
            else:
                contact_hold_s = 0.0
                contact_peak_n = 0.0

            before_qpos = np.asarray(
                data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7], dtype=np.float64
            ).copy()
            before_qvel = np.asarray(
                data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6], dtype=np.float64
            ).copy()
            was_detached = bool(buoy.detached)
            runtime.apply(dt)
            if buoy.detached and not was_detached:
                qpos_jump = float(
                    np.linalg.norm(
                        data.qpos[buoy.free_qposadr : buoy.free_qposadr + 7] - before_qpos
                    )
                )
                qvel_jump = float(
                    np.linalg.norm(
                        data.qvel[buoy.free_dofadr : buoy.free_dofadr + 6] - before_qvel
                    )
                )
                release_hold_s = contact_hold_s
                release_peak_n = contact_peak_n
                release_z = float(runtime._buoy_center_world(buoy)[2])
            mujoco.mj_step(model, data)
            # Preserve the solved impulse from the release step as evidence;
            # the runtime correctly made its decision from the pre-step
            # physical contact and did not wait for a later state-logic tick.
            trace.observe(mujoco, model, data)
            driver.sync()
            if buoy.detached:
                break

        root_geom = "mission_starboard_rake_root_probe"
        pvc_geom = f"{buoy_name}_pvc_pipe"
        fixture.require(trace.contains(root_geom, pvc_geom), f"{buoy_name} missed the rake root")
        fixture.require(buoy.detached, f"{buoy_name} did not detach on physical rake contact")
        fixture.require(
            release_hold_s <= dt + 1.0e-9,
            f"{buoy_name} did not release on first contact step: {release_hold_s:.6f}s",
        )
        fixture.require(qpos_jump <= 1.0e-6, f"{buoy_name} qpos jumped {qpos_jump:.3e}")
        fixture.require(qvel_jump <= 1.0e-6, f"{buoy_name} qvel jumped {qvel_jump:.3e}")
        fixture.require(
            buoy.eq_id < 0 or not bool(data.eq_active[buoy.eq_id]),
            f"{buoy_name} magnet equality stayed active",
        )
        _checkpoint(
            writer,
            name=f"rake_root_first_step_release_{buoy_name.rsplit('_', 2)[-2]}",
            runtime=runtime,
            base_id=base_id,
            buoys=(buoy,),
            trace=trace,
            metrics={
                "release_contact_hold_s": release_hold_s,
                "release_contact_peak_n": release_peak_n,
                "release_step_solved_contact_force_n": trace.peak_force(root_geom, pvc_geom),
                "release_qpos_jump_l2": qpos_jump,
                "release_qvel_jump_l2": qvel_jump,
                "release_center_z_m": release_z,
                "push_speed_mps": push_speed_mps,
            },
        )
        driver.sync(force=True)
        driver.hold()
        return {
            "release_z": release_z,
            "qpos_jump": qpos_jump,
            "qvel_jump": qvel_jump,
        }

    released = {
        "course_buoy_a_yellow_1": strike("course_buoy_a_yellow_1", verify_gap=True),
        "course_buoy_a_orange_1": strike("course_buoy_a_orange_1", verify_gap=False),
    }
    checked = {
        name: next(item for item in runtime.buoys if item.name == name) for name in released
    }
    target_z = {name: runtime._surface_target_center_z(buoy) for name, buoy in checked.items()}
    max_z = {name: float(runtime._buoy_center_world(buoy)[2]) for name, buoy in checked.items()}
    surface_step: dict[str, int] = {}
    post_surface_steps = max(1, int(np.ceil(4.0 / dt)))
    max_steps = max(1, int(np.ceil(90.0 / dt)))

    _set_vehicle_pose(
        mujoco,
        model,
        data,
        qposadr=base_qposadr,
        dofadr=base_dofadr,
        xyz=np.array([0.0, 0.0, -4.0]),
    )
    driver.stage(speed=ascent_speed)
    yellow = checked["course_buoy_a_yellow_1"]
    for step in range(max_steps):
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        fixture.require(np.all(np.isfinite(data.qpos)), "rake ascent produced non-finite qpos")
        fixture.require(np.all(np.isfinite(data.qvel)), "rake ascent produced non-finite qvel")
        yellow_xyz = runtime._buoy_center_world(yellow)
        driver.focus(yellow_xyz, distance=2.3, azimuth=90.0, elevation=-5.0)
        driver.sync()
        for name, buoy in checked.items():
            center_z = float(runtime._buoy_center_world(buoy)[2])
            max_z[name] = max(max_z[name], center_z)
            if buoy.surface_on_waterline and center_z >= target_z[name] - 0.005:
                surface_step.setdefault(name, step)
        if len(surface_step) == len(checked) and step - max(surface_step.values()) >= post_surface_steps:
            break

    metrics: dict[str, Any] = {"ascent_view_speed": ascent_speed, "buoys": {}}
    for name, buoy in checked.items():
        final_z = float(runtime._buoy_center_world(buoy)[2])
        final_vz = float(data.qvel[buoy.free_dofadr + 2])
        rise_m = final_z - released[name]["release_z"]
        fixture.require(name in surface_step, f"{name} never reached the waterline")
        fixture.require(rise_m > 8.0, f"{name} rose only {rise_m:.3f}m")
        fixture.require(
            abs(final_z - target_z[name]) <= 0.035,
            f"{name} did not settle at waterline: z={final_z:.3f}",
        )
        fixture.require(abs(final_vz) <= 0.20, f"{name} unstable at surface: vz={final_vz:.3f}")
        fixture.require(
            max_z[name] <= target_z[name] + 0.080,
            f"{name} overshot waterline: peak={max_z[name]:.3f}",
        )
        metrics["buoys"][name] = {
            "rise_m": rise_m,
            "target_z_m": target_z[name],
            "peak_z_m": max_z[name],
            "final_z_m": final_z,
            "final_vz_mps": final_vz,
        }
    driver.focus(runtime._buoy_center_world(yellow), distance=2.3, azimuth=90.0, elevation=-5.0)
    _checkpoint(
        writer,
        name="released_yellow_orange_reach_waterline",
        runtime=runtime,
        base_id=base_id,
        buoys=checked.values(),
        metrics=metrics,
    )
    driver.sync(force=True)
    driver.hold()


def _prepare_collector_pose(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    runtime: Any,
    item: Any,
    base_qposadr: int,
    base_dofadr: int,
    base_xy: Iterable[float],
) -> np.ndarray:
    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    target_z = runtime._surface_target_center_z(item)
    xy = np.asarray(tuple(base_xy), dtype=np.float64).reshape(2)
    _set_vehicle_pose(
        mujoco,
        model,
        data,
        qposadr=base_qposadr,
        dofadr=base_dofadr,
        xyz=np.array([xy[0], xy[1], target_z - hold[2]]),
    )
    return np.asarray(data.qpos[base_qposadr : base_qposadr + 7], dtype=np.float64).copy()


def _capture_through_mouth_visible(
    *,
    label: str,
    mujoco: Any,
    model: Any,
    data: Any,
    runtime: Any,
    driver: ViewerDriver,
    writer: EvidenceWriter,
    base_id: int,
    base_qposadr: int,
    base_dofadr: int,
    held_base_pose: np.ndarray,
    item: Any,
    expected_slot: int,
) -> dict[str, float]:
    """Visible equivalent of the shared physical-mouth fixture.

    Placement/coordinate helpers and every force/state transition come from
    ``check_buoy_collector_capture`` and ``CourseBuoyRuntime`` respectively.
    This wrapper only inserts viewer synchronization and evidence checkpoints.
    """

    dt = float(model.opt.timestep)
    hold = np.asarray(runtime.collector_net_hold_local, dtype=np.float64)
    capture_x = float(hold[0]) + 0.65 * float(runtime.collector_net_window_x_m)
    if not item.detached:
        runtime._detach(item, reason="viewer_acceptance_fixture", force_n=runtime.break_force_n)
    item.netting = False
    item.netted = False
    item.net_reverse_released = False
    item.net_score_released = False
    item.net_slot_index = -1
    if item.collector_eq_id >= 0:
        data.eq_active[item.collector_eq_id] = 0
    runtime._restore_buoy_collisions(item)
    runtime._set_netted_gate_collision(item, enabled=False)
    item_target_z = runtime._surface_target_center_z(item)
    fixture.place_free_buoy(
        mujoco,
        model,
        data,
        item.body_id,
        item.name,
        fixture.world_from_base_local(
            data,
            base_id,
            np.array([capture_x, hold[1], item_target_z - float(data.xpos[base_id, 2])]),
        ),
    )
    trace = ContactTrace(tokens=(item.name, "collector"))
    driver.stage(speed=2.0)
    driver.focus(data.xpos[base_id], distance=2.0, azimuth=120.0, elevation=-18.0)
    fixture.require(_capture_state(runtime, item) == "FREE", f"{item.name} did not start FREE")
    _checkpoint(
        writer,
        name=f"{label}_free_at_open_mouth",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        metrics={"fixture_initial_placement": True, "mouth_capture_x_local_m": capture_x},
    )
    driver.sync(force=True)
    driver.hold()

    for _ in range(fixture.runtime_detection_steps(runtime, model)):
        data.qpos[base_qposadr : base_qposadr + 7] = held_base_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        trace.observe(mujoco, model, data)
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        driver.sync()
        if item.netting:
            break
    fixture.require(item.netting and not item.netted, f"{item.name} did not enter NETTING")
    fixture.require(_capture_state(runtime, item) == "NETTING", f"{item.name} skipped NETTING")
    fixture.require_netted_gate_state(
        model,
        item,
        enabled=False,
        label=f"{item.name} NETTING gate",
    )
    _checkpoint(
        writer,
        name=f"{label}_netting_guided_at_mouth",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        trace=trace,
    )
    driver.sync(force=True)
    driver.hold()

    activation_qpos_jump = -1.0
    activation_qvel_jump = -1.0
    for _ in range(max(1, int(round(5.0 / dt)))):
        data.qpos[base_qposadr : base_qposadr + 7] = held_base_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        before_qpos = np.asarray(
            data.qpos[item.free_qposadr : item.free_qposadr + 7], dtype=np.float64
        ).copy()
        before_qvel = np.asarray(
            data.qvel[item.free_dofadr : item.free_dofadr + 6], dtype=np.float64
        ).copy()
        was_netted = bool(item.netted)
        runtime.apply(dt)
        if item.netted and not was_netted:
            activation_qpos_jump = float(
                np.linalg.norm(data.qpos[item.free_qposadr : item.free_qposadr + 7] - before_qpos)
            )
            activation_qvel_jump = float(
                np.linalg.norm(data.qvel[item.free_dofadr : item.free_dofadr + 6] - before_qvel)
            )
        mujoco.mj_step(model, data)
        trace.observe(mujoco, model, data)
        driver.focus(data.xpos[base_id], distance=2.0, azimuth=120.0, elevation=-18.0)
        driver.sync()
        if item.netted:
            break

    fixture.require(item.netted and not item.netting, f"{item.name} did not become NETTED")
    fixture.require(_capture_state(runtime, item) == "NETTED", f"{item.name} state is not NETTED")
    fixture.require(
        item.collector_eq_id >= 0 and bool(data.eq_active[item.collector_eq_id]),
        f"{item.name} collector equality did not activate",
    )
    fixture.require(activation_qpos_jump <= 1.0e-12, "collector activation changed qpos")
    fixture.require(activation_qvel_jump <= 1.0e-12, "collector activation changed qvel")
    fixture.require_netted_gate_state(
        model,
        item,
        enabled=True,
        label=f"{item.name} NETTED gate",
    )
    local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
    slot = runtime._collector_net_slot_local(item)
    fixture.require(
        float(np.linalg.norm(local - slot)) <= 0.11,
        f"{item.name} missed collector slot: local={local}, slot={slot}",
    )
    fixture.require(item.net_slot_index == expected_slot, f"{item.name} got wrong slot")
    _checkpoint(
        writer,
        name=f"{label}_netted_collector_equality_active",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        trace=trace,
        metrics={
            "collector_activation_qpos_jump_l2": activation_qpos_jump,
            "collector_activation_qvel_jump_l2": activation_qvel_jump,
            "slot_error_m": float(np.linalg.norm(local - slot)),
        },
    )
    driver.sync(force=True)
    driver.hold()
    return {
        "activation_qpos_jump": activation_qpos_jump,
        "activation_qvel_jump": activation_qvel_jump,
    }


def _run_carry_and_reverse(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    runtime: Any,
    driver: ViewerDriver,
    writer: EvidenceWriter,
    base_id: int,
    base_qposadr: int,
    base_dofadr: int,
    item: Any,
) -> None:
    dt = float(model.opt.timestep)
    held_base_z = float(data.qpos[base_qposadr + 2])
    trace = ContactTrace(tokens=(item.name, "collector"))
    start_local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
    max_slot_error = 0.0
    driver.stage(speed=2.0)
    for _ in range(max(1, int(round(2.5 / dt)))):
        data.qpos[base_qposadr + 2] = held_base_z
        fixture.set_base_local_linear_velocity(
            data, base_id, base_dofadr, np.array([0.30, 0.0, -0.02])
        )
        data.qvel[base_dofadr + 5] = 0.18
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        trace.observe(mujoco, model, data)
        local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
        max_slot_error = max(
            max_slot_error,
            float(np.linalg.norm(local - runtime._collector_net_slot_local(item))),
        )
        driver.focus(data.xpos[base_id], distance=2.2, azimuth=135.0, elevation=-18.0)
        driver.sync()
        fixture.require(item.netted, "forward/yaw carry released the red buoy")
        fixture.require(bool(data.eq_active[item.collector_eq_id]), "carry dropped collector equality")
    end_local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
    fixture.require(
        -0.33 <= float(end_local[0]) <= 0.31
        and abs(float(end_local[1])) <= 0.32
        and 0.15 <= float(end_local[2]) <= 0.62,
        f"carried red buoy left collector: local={end_local}",
    )
    _checkpoint(
        writer,
        name="red_forward_yaw_carry_retained_in_slot",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        trace=trace,
        metrics={
            "carry_duration_s": 2.5,
            "forward_speed_mps": 0.30,
            "yaw_rate_rps": 0.18,
            "start_local_xyz_m": _vec(start_local),
            "end_local_xyz_m": _vec(end_local),
            "max_slot_error_m": max_slot_error,
        },
    )
    driver.sync(force=True)
    driver.hold()

    # The reverse transition itself must only change equality/state; position
    # and velocity are sampled on both sides of runtime.apply, before mj_step.
    item.release_time_s = -10.0
    data.qvel[base_dofadr : base_dofadr + 6] = 0.0
    fixture.set_base_local_linear_velocity(data, base_id, base_dofadr, np.array([-0.30, 0.0, 0.0]))
    mujoco.mj_forward(model, data)
    trace = ContactTrace(tokens=(item.name, "collector"))
    trace.observe(mujoco, model, data)
    qpos_before = np.asarray(
        data.qpos[item.free_qposadr : item.free_qposadr + 7], dtype=np.float64
    ).copy()
    qvel_before = np.asarray(
        data.qvel[item.free_dofadr : item.free_dofadr + 6], dtype=np.float64
    ).copy()
    runtime.apply(dt)
    qpos_delta = np.asarray(data.qpos[item.free_qposadr : item.free_qposadr + 7]) - qpos_before
    qvel_delta = np.asarray(data.qvel[item.free_dofadr : item.free_dofadr + 6]) - qvel_before
    fixture.require(item.net_reverse_released and not item.netted, "reverse did not open mouth")
    fixture.require(not bool(data.eq_active[item.collector_eq_id]), "reverse left weld active")
    fixture.require(float(np.max(np.abs(qpos_delta))) <= 1.0e-9, "reverse changed buoy qpos")
    fixture.require(float(np.max(np.abs(qvel_delta))) <= 1.0e-9, "reverse changed buoy qvel")
    fixture.require_netted_gate_state(
        model,
        item,
        enabled=False,
        label="reverse release gate",
    )
    mujoco.mj_step(model, data)
    driver.stage(speed=2.0)
    driver.sync(force=True)
    _checkpoint(
        writer,
        name="red_deliberate_reverse_opens_mouth",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        trace=trace,
        metrics={
            "reverse_release_qpos_jump_l2": float(np.linalg.norm(qpos_delta)),
            "reverse_release_qpos_jump_linf": float(np.max(np.abs(qpos_delta))),
            "reverse_release_qvel_jump_l2": float(np.linalg.norm(qvel_delta)),
            "reverse_release_qvel_jump_linf": float(np.max(np.abs(qvel_delta))),
        },
    )
    driver.hold()

    mouth_clear_x = 0.315 + float(item.release_radius_m) + 0.015
    max_local_x = -float("inf")
    for _ in range(max(1, int(round(3.0 / dt)))):
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        fixture.set_base_local_linear_velocity(
            data, base_id, base_dofadr, np.array([-0.30, 0.0, 0.0])
        )
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        trace.observe(mujoco, model, data)
        local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
        max_local_x = max(max_local_x, float(local[0]))
        driver.focus(data.xpos[base_id], distance=2.2, azimuth=135.0, elevation=-18.0)
        driver.sync()
        if float(local[0]) >= mouth_clear_x:
            break
    local = fixture.base_local_from_world(data, base_id, runtime._buoy_center_world(item))
    fixture.require(
        float(local[0]) >= mouth_clear_x,
        f"red buoy did not exit open mouth: local={local}, clear={mouth_clear_x:.3f}",
    )
    if item.net_reverse_released:
        runtime.apply(dt)
    fixture.require(not item.net_reverse_released, "reverse no-recapture latch did not clear")
    fixture.require(not item.netted, "reverse-exited buoy was recaptured")
    fixture.require(not bool(data.eq_active[item.collector_eq_id]), "exited buoy weld reactivated")
    _checkpoint(
        writer,
        name="red_naturally_exits_open_mouth",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        trace=trace,
        metrics={"mouth_clear_x_m": mouth_clear_x, "max_local_x_m": max_local_x},
    )
    driver.sync(force=True)
    driver.hold()


def _run_score_stage(
    *,
    mujoco: Any,
    model: Any,
    data: Any,
    runtime: Any,
    driver: ViewerDriver,
    writer: EvidenceWriter,
    base_id: int,
    base_qposadr: int,
    base_dofadr: int,
    status_path: Path,
) -> None:
    dt = float(model.opt.timestep)
    item = next(buoy for buoy in runtime.buoys if buoy.name == "course_buoy_a_red_2")
    score_zone = np.asarray(runtime.score_zone_a, dtype=np.float64)
    score_probe = np.asarray(runtime.collector_net_score_probe_local, dtype=np.float64)
    status_path.write_text(
        json.dumps({"state": "SCORE_ZONE_TRANSIT", "score_zone": {"xyz": _vec(score_zone)}}),
        encoding="utf-8",
    )
    runtime.collector_net_score_release_status_path = status_path
    runtime.collector_net_score_release_enable = True
    held_pose = _prepare_collector_pose(
        mujoco=mujoco,
        model=model,
        data=data,
        runtime=runtime,
        item=item,
        base_qposadr=base_qposadr,
        base_dofadr=base_dofadr,
        base_xy=score_zone[:2] - score_probe[:2],
    )
    _capture_through_mouth_visible(
        label="score_red",
        mujoco=mujoco,
        model=model,
        data=data,
        runtime=runtime,
        driver=driver,
        writer=writer,
        base_id=base_id,
        base_qposadr=base_qposadr,
        base_dofadr=base_dofadr,
        held_base_pose=held_pose,
        item=item,
        expected_slot=0,
    )

    held_samples = 0
    driver.stage(speed=2.0)
    for _ in range(max(1, int(round(0.50 / dt)))):
        data.qpos[base_qposadr : base_qposadr + 7] = held_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        held_samples += 1
        fixture.require(item.netted, "score buoy released during SCORE_ZONE_TRANSIT")
        fixture.require(
            bool(data.eq_active[item.collector_eq_id]),
            "score buoy weld dropped before RELEASE",
        )
        driver.focus(data.xpos[base_id], distance=2.2, azimuth=135.0, elevation=-20.0)
        driver.sync()
    _checkpoint(
        writer,
        name="score_transit_weld_held_before_release",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        metrics={
            "fsm_state": "SCORE_ZONE_TRANSIT",
            "held_physics_steps": held_samples,
            "collector_inside_score_zone": bool(runtime._collector_inside_score_zone()),
        },
    )
    driver.sync(force=True)
    driver.hold()

    status_path.write_text(
        json.dumps({"state": "RELEASE", "score_zone": {"xyz": _vec(score_zone)}}),
        encoding="utf-8",
    )
    qpos_jump = -1.0
    qvel_jump = -1.0
    release_seen = False
    for _ in range(max(1, int(round(1.0 / dt)))):
        data.qpos[base_qposadr : base_qposadr + 7] = held_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        before_qpos = np.asarray(
            data.qpos[item.free_qposadr : item.free_qposadr + 7], dtype=np.float64
        ).copy()
        before_qvel = np.asarray(
            data.qvel[item.free_dofadr : item.free_dofadr + 6], dtype=np.float64
        ).copy()
        was_released = bool(item.net_score_released)
        runtime.apply(dt)
        if item.net_score_released and not was_released:
            qpos_jump = float(
                np.linalg.norm(data.qpos[item.free_qposadr : item.free_qposadr + 7] - before_qpos)
            )
            qvel_jump = float(
                np.linalg.norm(data.qvel[item.free_dofadr : item.free_dofadr + 6] - before_qvel)
            )
            release_seen = True
        mujoco.mj_step(model, data)
        driver.sync()
        if release_seen:
            break
    fixture.require(release_seen, "score buoy did not release in RELEASE state")
    fixture.require(item.net_score_released and not item.netted, "score release state is invalid")
    fixture.require(_capture_state(runtime, item) == "SCORE_RELEASED", "missing SCORE_RELEASED")
    fixture.require(not bool(data.eq_active[item.collector_eq_id]), "score weld stayed active")
    fixture.require(qpos_jump <= 1.0e-12, "score equality release changed qpos")
    fixture.require(qvel_jump <= 1.0e-12, "score equality release changed qvel")
    fixture.require_netted_gate_state(
        model,
        item,
        enabled=False,
        label="score release gate",
    )
    _checkpoint(
        writer,
        name="score_release_state_only_releases_weld",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        metrics={
            "fsm_state": "RELEASE",
            "score_release_qpos_jump_l2": qpos_jump,
            "score_release_qvel_jump_l2": qvel_jump,
            "score_release_velocity_is_runtime_commanded": False,
        },
    )
    driver.sync(force=True)
    driver.hold()

    settle_s = max(5.2, float(runtime.collector_net_score_settle_s) + 0.5)
    driver.stage(speed=4.0)
    for _ in range(max(1, int(round(settle_s / dt)))):
        data.qpos[base_qposadr : base_qposadr + 7] = held_pose
        data.qvel[base_dofadr : base_dofadr + 6] = 0.0
        runtime.apply(dt)
        mujoco.mj_step(model, data)
        fixture.require(np.all(np.isfinite(data.qacc)), "score settle produced non-finite qacc")
        driver.focus(data.xpos[base_id], distance=2.2, azimuth=135.0, elevation=-20.0)
        driver.sync()
    center = runtime._buoy_center_world(item)
    horizontal_error = float(np.linalg.norm(center[:2] - score_zone[:2]))
    fixture.require(
        horizontal_error <= runtime.collector_net_score_radius_m,
        f"released red buoy settled outside score zone: error={horizontal_error:.3f}",
    )
    _checkpoint(
        writer,
        name="score_released_buoy_remains_in_selected_zone",
        runtime=runtime,
        base_id=base_id,
        buoys=(item,),
        metrics={
            "score_zone_xyz_m": _vec(score_zone),
            "horizontal_error_m": horizontal_error,
            "allowed_radius_m": float(runtime.collector_net_score_radius_m),
        },
    )
    driver.sync(force=True)
    driver.hold()


def _resolve_output(value: str | None) -> Path:
    GENERATED.mkdir(parents=True, exist_ok=True)
    if value is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return GENERATED / f"buoy_viewer_acceptance_{stamp}.jsonl"
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = GENERATED / candidate
    candidate = candidate.resolve()
    generated = GENERATED.resolve()
    if not candidate.is_relative_to(generated):
        raise ValueError(f"evidence output must stay under {generated}: {candidate}")
    return candidate


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        help="JSONL filename under current/generated (default: timestamped filename)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run every deterministic stage without opening a viewer (CI use)",
    )
    parser.add_argument(
        "--dry-contract",
        action="store_true",
        help="only load the canonical scene/runtime and verify required names",
    )
    parser.add_argument(
        "--render-hz",
        type=_bounded_positive,
        default=30.0,
        help="bounded viewer refresh rate (5..60, default: 30)",
    )
    parser.add_argument(
        "--ascent-view-speed",
        type=_bounded_positive,
        default=12.0,
        help="viewer time acceleration for the 8.5 m ascent (default: 12x)",
    )
    parser.add_argument(
        "--checkpoint-hold-s",
        type=_finite_nonnegative,
        default=0.45,
        help="wall-clock pause at each visible checkpoint (default: 0.45)",
    )
    parser.add_argument(
        "--hold-final-s",
        type=_finite_nonnegative,
        default=3.0,
        help="finite final viewer hold before exit (default: 3)",
    )
    parser.add_argument(
        "--keep-open",
        action="store_true",
        help="after successful finite run, keep the viewer open until manually closed",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.keep_open and (args.headless or args.dry_contract):
        raise SystemExit("--keep-open requires the default viewer mode")
    output = _resolve_output(args.output)
    writer = EvidenceWriter(output)
    passed = False
    try:
        import mujoco
        import mujoco.viewer as mujoco_viewer

        # Prefer XWayland on the desktop used for acceptance runs.  GLFW's
        # automatic Wayland backend currently segfaults while the passive
        # MuJoCo viewer is being destroyed, after all evidence has already
        # been flushed.  Selecting X11 before glfwInit both makes the window
        # capturable with xwd and gives the harness a clean exit status.
        viewer_platform = "headless"
        if not args.headless and not args.dry_contract and os.environ.get("DISPLAY"):
            import glfw

            glfw_build = glfw.get_version_string() or b""
            if (
                hasattr(glfw, "PLATFORM")
                and hasattr(glfw, "PLATFORM_X11")
                and b"X11" in glfw_build
            ):
                glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)
                viewer_platform = "x11"
            else:
                # The locally installed glfw wheel is Wayland-only even
                # though it exports PLATFORM_X11 constants.  Do not select an
                # unavailable backend; the acceptance still runs visibly.
                viewer_platform = "automatic_wayland"

        fixture.require(
            fixture.SCENE.resolve() == SCENE.resolve(),
            f"shared fixture scene mismatch: {fixture.SCENE} != {SCENE}",
        )
        model = mujoco.MjModel.from_xml_path(str(SCENE))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        runtime = fixture.make_runtime(mujoco, model, data)
        base_id = fixture.body_id(mujoco, model, "base_link")
        world_joint = fixture.joint_id(mujoco, model, "world_joint")
        base_qposadr = int(model.jnt_qposadr[world_joint])
        base_dofadr = int(model.jnt_dofadr[world_joint])
        required_buoys = (
            "course_buoy_a_yellow_1",
            "course_buoy_a_orange_1",
            "course_buoy_a_red_1",
            "course_buoy_a_red_2",
        )
        available = {buoy.name for buoy in runtime.buoys}
        fixture.require(set(required_buoys) <= available, "canonical scene is missing required buoys")
        fixture.require(runtime.collector_net_enable, "canonical collector runtime is disabled")
        fixture.require(
            runtime.vehicle_release_probe_geom_ids,
            "canonical scene has no physical rake release probes",
        )
        writer.emit(
            "start",
            scene=str(SCENE.resolve()),
            scene_sha256=_scene_sha256(),
            canonical_scene=True,
            mujoco_version=str(mujoco.__version__),
            timestep_s=float(model.opt.timestep),
            body_count=int(model.nbody),
            geom_count=int(model.ngeom),
            equality_count=int(model.neq),
            viewer_requested=not args.headless and not args.dry_contract,
            viewer_platform=viewer_platform,
            dry_contract=bool(args.dry_contract),
        )

        if args.dry_contract:
            writer.emit(
                "contract",
                passed=True,
                required_buoys=list(required_buoys),
                runtime_buoy_count=len(runtime.buoys),
                rake_release_probe_count=len(runtime.vehicle_release_probe_geom_ids),
                contact_release_hold_s=float(runtime.contact_release_hold_s),
                collector_net_enabled=bool(runtime.collector_net_enable),
                score_release_phase_gate=bool(runtime.collector_net_score_release_phase_gate),
            )
            passed = True
            writer.emit("result", passed=True, evidence_path=str(output), dry_contract=True)
            print(f"buoy viewer acceptance dry contract: ok\nevidence: {output}")
            return 0

        viewer_context = (
            nullcontext(None)
            if args.headless
            else mujoco_viewer.launch_passive(
                model,
                data,
                show_left_ui=False,
                show_right_ui=False,
            )
        )
        with viewer_context as viewer:
            driver = ViewerDriver(
                mujoco=mujoco,
                model=model,
                data=data,
                viewer=viewer,
                render_hz=args.render_hz,
                checkpoint_hold_s=args.checkpoint_hold_s,
            )
            _run_rake_stages(
                mujoco=mujoco,
                model=model,
                data=data,
                runtime=runtime,
                driver=driver,
                writer=writer,
                base_id=base_id,
                base_qposadr=base_qposadr,
                base_dofadr=base_dofadr,
                ascent_speed=min(30.0, max(1.0, float(args.ascent_view_speed))),
            )

            runtime.collector_net_score_release_enable = False
            red = next(item for item in runtime.buoys if item.name == "course_buoy_a_red_1")
            held_pose = _prepare_collector_pose(
                mujoco=mujoco,
                model=model,
                data=data,
                runtime=runtime,
                item=red,
                base_qposadr=base_qposadr,
                base_dofadr=base_dofadr,
                base_xy=(0.0, 0.0),
            )
            _capture_through_mouth_visible(
                label="red",
                mujoco=mujoco,
                model=model,
                data=data,
                runtime=runtime,
                driver=driver,
                writer=writer,
                base_id=base_id,
                base_qposadr=base_qposadr,
                base_dofadr=base_dofadr,
                held_base_pose=held_pose,
                item=red,
                expected_slot=0,
            )
            _run_carry_and_reverse(
                mujoco=mujoco,
                model=model,
                data=data,
                runtime=runtime,
                driver=driver,
                writer=writer,
                base_id=base_id,
                base_qposadr=base_qposadr,
                base_dofadr=base_dofadr,
                item=red,
            )

            with tempfile.TemporaryDirectory(
                prefix="buoy_viewer_score_", dir=str(GENERATED)
            ) as temporary:
                _run_score_stage(
                    mujoco=mujoco,
                    model=model,
                    data=data,
                    runtime=runtime,
                    driver=driver,
                    writer=writer,
                    base_id=base_id,
                    base_qposadr=base_qposadr,
                    base_dofadr=base_dofadr,
                    status_path=Path(temporary) / "mission_fsm_status.json",
                )

            passed = True
            writer.emit(
                "result",
                passed=True,
                evidence_path=str(output),
                final_sim_time_s=float(data.time),
                viewer=not args.headless,
            )
            print(f"buoy viewer acceptance: ok\nevidence: {output}", flush=True)
            driver.hold(min(60.0, float(args.hold_final_s)))
            if args.keep_open:
                print("[acceptance] --keep-open active; close the MuJoCo window to exit", flush=True)
                driver.keep_open()
        return 0
    except BaseException as exc:
        try:
            writer.emit(
                "result",
                passed=False,
                evidence_path=str(output),
                error_type=type(exc).__name__,
                error=str(exc),
                traceback=traceback.format_exc(),
            )
        finally:
            print(f"buoy viewer acceptance: FAILED ({exc})\nevidence: {output}", file=sys.stderr)
        raise
    finally:
        writer.close()
        if not passed:
            print("[acceptance] failure evidence was flushed", file=sys.stderr, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
