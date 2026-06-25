"""Competition-course buoy force and magnetic release runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import Any, Callable

import numpy as np


@dataclass
class CourseBuoy:
    name: str
    body_id: int
    attach_site_id: int
    magnet_site_id: int
    eq_id: int
    free_qposadr: int
    free_dofadr: int
    geom_ids: frozenset[int]
    projection_geom_ids: tuple[int, ...]
    has_magnet: bool
    detached: bool = False
    surface_on_waterline: bool = False
    release_time_s: float = -1.0
    last_runtime_wrench: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))


@dataclass
class CourseBuoyRuntime:
    """Apply per-buoy float force and a breakable magnetic hold."""

    mujoco_module: Any
    model: Any
    data: Any
    buoys: list[CourseBuoy]
    vehicle_geom_ids: frozenset[int]
    buoyancy_n: float
    vertical_damping_nspm: float
    water_linear_drag_nspm: float
    water_quadratic_drag_nspm2: float
    water_angular_drag_nmsprad: float
    float_half_height_m: float
    surface_spring_npm: float
    surface_capture_band_m: float
    release_stabilize_s: float
    release_max_down_speed_mps: float
    break_force_n: float
    magnet_stiffness_npm: float
    magnet_damping_nspm: float
    water_surface_z: float
    track_csv_path: Path | None
    track_interval_s: float
    log: Callable[[str], None]
    _last_track_time_s: float = -1.0
    _track_header_written: bool = False

    @classmethod
    def from_model(
        cls,
        *,
        mujoco_module: Any,
        model: Any,
        data: Any,
        water_surface_z: float,
        env_float: Callable[[str, float], float],
        env_flag: Callable[[str, bool], bool],
        log: Callable[[str], None],
    ) -> "CourseBuoyRuntime":
        if not env_flag("UUV_COURSE_BUOYS_ENABLE", True):
            return cls._empty(model=model, data=data, water_surface_z=water_surface_z, log=log)

        obj_body = mujoco_module.mjtObj.mjOBJ_BODY
        obj_geom = mujoco_module.mjtObj.mjOBJ_GEOM
        obj_site = mujoco_module.mjtObj.mjOBJ_SITE
        obj_equality = mujoco_module.mjtObj.mjOBJ_EQUALITY
        vehicle_geom_ids = cls._body_subtree_geom_ids(
            mujoco_module=mujoco_module,
            model=model,
            root_body_name="base_link",
        )
        buoys: list[CourseBuoy] = []
        for body_id in range(model.nbody):
            body_name = mujoco_module.mj_id2name(model, obj_body, body_id) or ""
            if not body_name.startswith("course_buoy_") or not body_name.endswith("_float"):
                continue
            prefix = body_name.removesuffix("_float")
            attach_site_id = mujoco_module.mj_name2id(model, obj_site, f"{prefix}_attach_site")
            magnet_site_id = mujoco_module.mj_name2id(model, obj_site, f"{prefix}_magnet_site")
            eq_id = mujoco_module.mj_name2id(model, obj_equality, f"{prefix}_magnet_weld")
            free_qposadr, free_dofadr = cls._free_joint_addresses(
                mujoco_module=mujoco_module,
                model=model,
                body_id=int(body_id),
            )
            geom_ids = frozenset(
                int(geom_id)
                for geom_id in range(model.ngeom)
                if int(model.geom_bodyid[geom_id]) == int(body_id)
            )
            projection_geom_ids = tuple(
                int(geom_id)
                for geom_id in (
                    mujoco_module.mj_name2id(model, obj_geom, f"{prefix}_surface_projection"),
                    mujoco_module.mj_name2id(model, obj_geom, f"{prefix}_surface_projection_outline"),
                )
                if geom_id >= 0
            )
            has_magnet = attach_site_id >= 0 and magnet_site_id >= 0
            buoys.append(
                CourseBuoy(
                    name=prefix,
                    body_id=int(body_id),
                    attach_site_id=int(attach_site_id),
                    magnet_site_id=int(magnet_site_id),
                    eq_id=int(eq_id),
                    free_qposadr=free_qposadr,
                    free_dofadr=free_dofadr,
                    geom_ids=geom_ids,
                    projection_geom_ids=projection_geom_ids,
                    has_magnet=has_magnet,
                    detached=not has_magnet,
                    surface_on_waterline=not has_magnet,
                )
            )

        runtime = cls(
            mujoco_module=mujoco_module,
            model=model,
            data=data,
            buoys=buoys,
            vehicle_geom_ids=vehicle_geom_ids,
            buoyancy_n=float(env_float("UUV_COURSE_BUOY_BUOYANCY_N", 0.98)),
            vertical_damping_nspm=float(env_float("UUV_COURSE_BUOY_VERTICAL_DAMPING_NSPM", 0.65)),
            water_linear_drag_nspm=float(env_float("UUV_COURSE_BUOY_WATER_LINEAR_DRAG_NSPM", 0.25)),
            water_quadratic_drag_nspm2=float(env_float("UUV_COURSE_BUOY_WATER_QUADRATIC_DRAG_NSPM2", 0.45)),
            water_angular_drag_nmsprad=float(env_float("UUV_COURSE_BUOY_WATER_ANGULAR_DRAG_NMSPRAD", 0.0008)),
            float_half_height_m=float(env_float("UUV_COURSE_BUOY_FLOAT_HALF_HEIGHT_M", 0.085)),
            surface_spring_npm=float(env_float("UUV_COURSE_BUOY_SURFACE_SPRING_NPM", 2.0)),
            surface_capture_band_m=float(env_float("UUV_COURSE_BUOY_SURFACE_CAPTURE_BAND_M", 0.25)),
            release_stabilize_s=float(env_float("UUV_COURSE_BUOY_RELEASE_STABILIZE_S", 0.0)),
            release_max_down_speed_mps=float(env_float("UUV_COURSE_BUOY_RELEASE_MAX_DOWN_SPEED_MPS", 0.05)),
            break_force_n=float(env_float("UUV_COURSE_BUOY_MAGNET_BREAK_N", 15.0)),
            magnet_stiffness_npm=float(env_float("UUV_COURSE_BUOY_MAGNET_STIFFNESS_NPM", 1200.0)),
            magnet_damping_nspm=float(env_float("UUV_COURSE_BUOY_MAGNET_DAMPING_NSPM", 8.0)),
            water_surface_z=float(water_surface_z),
            track_csv_path=cls._track_csv_path(env_flag("UUV_COURSE_BUOY_TRACK_CSV_ENABLE", True)),
            track_interval_s=float(env_float("UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S", 0.25)),
            log=log,
        )
        if buoys:
            log(
                "[course] buoys enabled: "
                f"count={len(buoys)}, full_immersion_net_lift={runtime.buoyancy_n:.3f}N, "
                f"magnet_break={runtime.break_force_n:.3f}N"
            )
            runtime._log_float_contract()
            runtime._log_track_contract()
        return runtime

    @classmethod
    def _empty(
        cls,
        *,
        model: Any,
        data: Any,
        water_surface_z: float,
        log: Callable[[str], None],
    ) -> "CourseBuoyRuntime":
        return cls(
            mujoco_module=None,
            model=model,
            data=data,
            buoys=[],
            vehicle_geom_ids=frozenset(),
            buoyancy_n=0.0,
            vertical_damping_nspm=0.0,
            water_linear_drag_nspm=0.0,
            water_quadratic_drag_nspm2=0.0,
            water_angular_drag_nmsprad=0.0,
            float_half_height_m=0.085,
            surface_spring_npm=2.0,
            surface_capture_band_m=0.25,
            release_stabilize_s=0.0,
            release_max_down_speed_mps=0.0,
            break_force_n=0.0,
            magnet_stiffness_npm=0.0,
            magnet_damping_nspm=0.0,
            water_surface_z=float(water_surface_z),
            track_csv_path=None,
            track_interval_s=0.25,
            log=log,
        )

    def apply(self, dt: float) -> None:
        if not self.buoys:
            return

        for buoy in self.buoys:
            self._clear_persisted_runtime_wrench(buoy)
            if buoy.has_magnet and not buoy.detached:
                self._release_if_break_force_exceeded(buoy)

            wrench = np.zeros(6, dtype=np.float64)
            wrench += self._float_buoyancy_wrench(buoy)
            wrench += self._water_drag_wrench(buoy)
            if buoy.has_magnet and not buoy.detached:
                wrench += self._magnet_hold_wrench(buoy, dt)

            self.data.xfrc_applied[buoy.body_id, :] += wrench
            buoy.last_runtime_wrench = wrench
            self._apply_surface_float_guard(buoy)
        self._write_tracking_sample()

    def _clear_persisted_runtime_wrench(self, buoy: CourseBuoy) -> None:
        last = buoy.last_runtime_wrench
        if not np.any(last):
            return

        current = np.array(self.data.xfrc_applied[buoy.body_id, :], dtype=np.float64)
        active = np.abs(last) > 1e-9
        if not bool(np.any(active)):
            buoy.last_runtime_wrench[:] = 0.0
            return
        same_direction = np.sign(current[active]) == np.sign(last[active])
        still_present = np.abs(current[active]) >= (0.5 * np.abs(last[active]))
        removable = active.copy()
        removable[active] = same_direction & still_present
        if bool(np.any(removable)):
            self.data.xfrc_applied[buoy.body_id, removable] -= last[removable]
        buoy.last_runtime_wrench[:] = 0.0

    def _float_buoyancy_wrench(self, buoy: CourseBuoy) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        if self.buoyancy_n <= 0.0:
            return wrench

        center_z = float(self.data.xipos[buoy.body_id, 2])
        if not self._touches_float_waterline(buoy, center_z):
            return wrench

        velocity_z = float(self._body_linear_velocity(buoy.body_id)[2])
        target_z = self._surface_target_center_z(buoy)
        neutral_upthrust_n = self._body_weight_n(buoy)
        if self._has_vehicle_contact(buoy) and center_z < target_z:
            wrench[2] = neutral_upthrust_n + self.buoyancy_n
            return wrench

        depth_error_m = target_z - center_z
        net_lift_n = (self.surface_spring_npm * depth_error_m) - (self.vertical_damping_nspm * velocity_z)
        upthrust_n = neutral_upthrust_n + float(np.clip(net_lift_n, 0.0, self.buoyancy_n))
        wrench[2] = upthrust_n
        return wrench

    def _water_drag_wrench(self, buoy: CourseBuoy) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        center_z = float(self.data.xipos[buoy.body_id, 2])
        if not self._touches_float_waterline(buoy, center_z):
            return wrench

        velocity = self._body_linear_velocity(buoy.body_id)
        if self._has_vehicle_contact(buoy):
            velocity = velocity.copy()
            velocity[2] = 0.0
        speed = float(np.linalg.norm(velocity))
        if self.water_linear_drag_nspm > 0.0:
            wrench[0:3] -= self.water_linear_drag_nspm * velocity
        if self.water_quadratic_drag_nspm2 > 0.0 and speed > 0.0:
            wrench[0:3] -= self.water_quadratic_drag_nspm2 * speed * velocity

        angular_velocity = self._body_angular_velocity(buoy.body_id)
        if self.water_angular_drag_nmsprad > 0.0:
            wrench[3:6] -= self.water_angular_drag_nmsprad * angular_velocity
        return wrench

    def _touches_float_waterline(self, buoy: CourseBuoy, center_z: float) -> bool:
        return float(center_z) <= self.water_surface_z + self.float_half_height_m

    def _surface_target_center_z(self, buoy: CourseBuoy) -> float:
        if self.buoyancy_n <= 0.0:
            return self.water_surface_z - self.float_half_height_m
        weight_n = self._body_weight_n(buoy)
        full_upthrust_n = self._full_immersion_upthrust_n(buoy)
        equilibrium_fraction = float(np.clip(weight_n / full_upthrust_n, 0.0, 1.0))
        return self.water_surface_z + self.float_half_height_m - 2.0 * self.float_half_height_m * equilibrium_fraction

    def _body_weight_n(self, buoy: CourseBuoy) -> float:
        gravity_z = float(np.asarray(self.model.opt.gravity, dtype=np.float64)[2])
        return float(self.model.body_mass[buoy.body_id]) * abs(gravity_z)

    def _full_immersion_upthrust_n(self, buoy: CourseBuoy) -> float:
        # The course spec's 0.98N is treated as net upward lift at full
        # immersion. MuJoCo still applies gravity separately, so the applied
        # upward water force must include the body's weight.
        return self._body_weight_n(buoy) + self.buoyancy_n

    def _log_float_contract(self) -> None:
        masses = [float(self.model.body_mass[buoy.body_id]) for buoy in self.buoys]
        weights = [self._body_weight_n(buoy) for buoy in self.buoys]
        upthrusts = [self._full_immersion_upthrust_n(buoy) for buoy in self.buoys]
        targets = [self._surface_target_center_z(buoy) for buoy in self.buoys]
        starts = [float(self.data.xipos[buoy.body_id, 2]) for buoy in self.buoys]
        samples = ", ".join(
            f"{buoy.name}:mass={float(self.model.body_mass[buoy.body_id]):.3f}kg,"
            f"weight={self._body_weight_n(buoy):.3f}N,"
            f"upthrust_full={self._full_immersion_upthrust_n(buoy):.3f}N,"
            f"target_z={self._surface_target_center_z(buoy):+.3f}m"
            for buoy in self.buoys[:4]
        )
        self.log(
            "[course] float contract: "
            f"mass_range={min(masses):.3f}..{max(masses):.3f}kg, "
            f"weight_range={min(weights):.3f}..{max(weights):.3f}N, "
            f"full_immersion_upthrust={min(upthrusts):.3f}..{max(upthrusts):.3f}N, "
            f"net_lift_full_immersion={self.buoyancy_n:.3f}N, "
            f"target_center_z={min(targets):+.3f}..{max(targets):+.3f}m, "
            f"initial_center_z={min(starts):+.3f}..{max(starts):+.3f}m, "
            f"waterline_model=capped_surface_spring, surface_spring={self.surface_spring_npm:.3f}N/m, "
            f"surface_capture_band={self.surface_capture_band_m:.3f}m, "
            f"water_drag=linear{self.water_linear_drag_nspm:.3f}/vertical{self.vertical_damping_nspm:.3f}"
            f"/quadratic{self.water_quadratic_drag_nspm2:.3f}, "
            f"samples=[{samples}]"
        )

    def _log_track_contract(self) -> None:
        if self.track_csv_path is None:
            self.log("[course] buoy live tracking disabled")
            return
        self.log(
            "[course] buoy live tracking: "
            f"path={self.track_csv_path}, interval={max(self.track_interval_s, 0.0):.3f}s"
        )

    @staticmethod
    def _track_csv_path(enabled: bool) -> Path | None:
        if not enabled:
            return None
        raw_path = os.environ.get("UUV_COURSE_BUOY_TRACK_CSV", "").strip()
        if raw_path:
            return Path(raw_path).expanduser()
        workspace = Path(__file__).resolve().parents[4]
        return workspace / "outputs" / "course_buoy_tracking" / "live.csv"

    def _write_tracking_sample(self) -> None:
        if self.track_csv_path is None:
            return
        now_s = float(getattr(self.data, "time", 0.0))
        interval_s = max(float(self.track_interval_s), 0.0)
        if self._last_track_time_s >= 0.0 and now_s < self._last_track_time_s + interval_s:
            return
        self._last_track_time_s = now_s

        path = self.track_csv_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            if not self._track_header_written:
                path.write_text(
                    "time_s,name,x_m,y_m,z_m,vz_mps,target_z_m,detached,on_waterline,force_z_n\n",
                    encoding="utf-8",
                )
                self._track_header_written = True
            with path.open("a", encoding="utf-8") as file:
                for buoy in self.buoys:
                    velocity_z = float(self.data.qvel[buoy.free_dofadr + 2]) if buoy.free_dofadr >= 0 else 0.0
                    force_z = float(self.data.xfrc_applied[buoy.body_id, 2])
                    pos = np.asarray(self.data.xipos[buoy.body_id], dtype=np.float64)
                    file.write(
                        f"{now_s:.3f},{buoy.name},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                        f"{velocity_z:.4f},{self._surface_target_center_z(buoy):.4f},"
                        f"{int(buoy.detached)},{int(buoy.surface_on_waterline)},{force_z:.4f}\n"
                    )
        except OSError as exc:
            self.log(f"[course] buoy live tracking disabled after write error: {exc}")
            self.track_csv_path = None

    def _limit_release_velocity(self, buoy: CourseBuoy) -> None:
        if not buoy.has_magnet or buoy.free_qposadr < 0 or buoy.free_dofadr < 0:
            return
        if buoy.release_time_s < 0.0 or self.release_stabilize_s <= 0.0:
            return
        if float(self.data.time) > buoy.release_time_s + self.release_stabilize_s:
            return
        center_z = float(self.data.qpos[buoy.free_qposadr + 2])
        if center_z >= self._surface_target_center_z(buoy):
            return
        velocity_z = float(self.data.qvel[buoy.free_dofadr + 2])
        min_velocity_z = -abs(self.release_max_down_speed_mps)
        if velocity_z < min_velocity_z:
            self.data.qvel[buoy.free_dofadr + 2] = min_velocity_z

    def _apply_surface_float_guard(self, buoy: CourseBuoy) -> None:
        """Keep released floats on the waterline without locking horizontal motion."""

        if not buoy.detached or buoy.free_qposadr < 0 or buoy.free_dofadr < 0:
            return

        target_z = self._surface_target_center_z(buoy)
        center_z = float(self.data.qpos[buoy.free_qposadr + 2])
        if center_z < target_z - self.surface_capture_band_m:
            buoy.surface_on_waterline = False
            return
        if self._has_vehicle_contact(buoy):
            return

        if not buoy.surface_on_waterline:
            buoy.surface_on_waterline = True

        velocity_z = float(self.data.qvel[buoy.free_dofadr + 2])
        emergency_floor_z = target_z - 0.030
        if center_z < emergency_floor_z and velocity_z < 0.0:
            self.data.qvel[buoy.free_dofadr + 2] = 0.0
        if center_z < emergency_floor_z:
            self.data.qpos[buoy.free_qposadr + 2] = emergency_floor_z

    def _has_vehicle_contact(self, buoy: CourseBuoy) -> bool:
        if not buoy.geom_ids or not self.vehicle_geom_ids:
            return False
        for contact_id in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            buoy_hit = geom1 in buoy.geom_ids or geom2 in buoy.geom_ids
            vehicle_hit = geom1 in self.vehicle_geom_ids or geom2 in self.vehicle_geom_ids
            if buoy_hit and vehicle_hit:
                return True
        return False

    def _magnet_hold_wrench(self, buoy: CourseBuoy, dt: float) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        if buoy.eq_id >= 0:
            return wrench
        attach_pos = np.array(self.data.site_xpos[buoy.attach_site_id], dtype=np.float64)
        magnet_pos = np.array(self.data.site_xpos[buoy.magnet_site_id], dtype=np.float64)
        displacement = attach_pos - magnet_pos
        velocity = self._body_linear_velocity(buoy.body_id)
        damping = self.magnet_damping_nspm * velocity if dt > 0.0 else 0.0
        magnet_force = -(self.magnet_stiffness_npm * displacement + damping)
        force_norm = float(np.linalg.norm(magnet_force))
        if force_norm >= self.break_force_n:
            self._detach(buoy, reason="spring", force_n=force_norm)
            return wrench
        wrench[0:3] = magnet_force
        return wrench

    def _release_if_break_force_exceeded(self, buoy: CourseBuoy) -> None:
        force_n, reason = self._release_force_sample(buoy)
        if force_n >= self.break_force_n:
            self._detach(buoy, reason=reason, force_n=force_n)

    def _release_force_sample(self, buoy: CourseBuoy) -> tuple[float, str]:
        # Do not use equality/weld solver force here. It includes the magnet's
        # own support force and can exceed 15N immediately after startup.
        samples = [
            (self._external_force_norm(buoy), "external"),
            (self._contact_force_norm(buoy), "contact"),
        ]
        return max(samples, key=lambda item: item[0])

    def _external_force_norm(self, buoy: CourseBuoy) -> float:
        force = np.array(self.data.xfrc_applied[buoy.body_id, 0:3], dtype=np.float64)
        return float(np.linalg.norm(force))

    def _contact_force_norm(self, buoy: CourseBuoy) -> float:
        if self.mujoco_module is None or not buoy.geom_ids:
            return 0.0
        contact_force = getattr(self.mujoco_module, "mj_contactForce", None)
        if contact_force is None:
            return 0.0
        force = np.zeros(6, dtype=np.float64)
        max_force = 0.0
        for contact_id in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            buoy_hit = geom1 in buoy.geom_ids or geom2 in buoy.geom_ids
            vehicle_hit = geom1 in self.vehicle_geom_ids or geom2 in self.vehicle_geom_ids
            if not (buoy_hit and vehicle_hit):
                continue
            contact_force(self.model, self.data, contact_id, force)
            max_force = max(max_force, float(np.linalg.norm(force[0:3])))
        return max_force

    def _detach(self, buoy: CourseBuoy, *, reason: str, force_n: float) -> None:
        if buoy.detached:
            return
        buoy.detached = True
        buoy.release_time_s = float(getattr(self.data, "time", 0.0))
        buoy.surface_on_waterline = False
        if buoy.eq_id >= 0 and hasattr(self.data, "eq_active"):
            self.data.eq_active[buoy.eq_id] = 0
        self._limit_release_velocity(buoy)
        self._hide_surface_projection(buoy)
        self.log(f"[course] magnet detached: {buoy.name} reason={reason} force={force_n:.3f}N")

    def _hide_surface_projection(self, buoy: CourseBuoy) -> None:
        for geom_id in buoy.projection_geom_ids:
            self.model.geom_rgba[int(geom_id), 3] = 0.0

    def _body_linear_velocity(self, body_id: int) -> np.ndarray:
        cvel = getattr(self.data, "cvel", None)
        if cvel is None:
            return np.zeros(3, dtype=np.float64)
        return np.array(cvel[body_id, 3:6], dtype=np.float64)

    def _body_angular_velocity(self, body_id: int) -> np.ndarray:
        cvel = getattr(self.data, "cvel", None)
        if cvel is None:
            return np.zeros(3, dtype=np.float64)
        return np.array(cvel[body_id, 0:3], dtype=np.float64)

    @staticmethod
    def _body_subtree_geom_ids(*, mujoco_module: Any, model: Any, root_body_name: str) -> frozenset[int]:
        obj_body = mujoco_module.mjtObj.mjOBJ_BODY
        root_body_id = int(mujoco_module.mj_name2id(model, obj_body, root_body_name))
        if root_body_id < 0:
            return frozenset()

        body_ids = {root_body_id}
        changed = True
        while changed:
            changed = False
            for body_id in range(model.nbody):
                parent_id = int(model.body_parentid[body_id])
                if int(body_id) not in body_ids and parent_id in body_ids:
                    body_ids.add(int(body_id))
                    changed = True

        return frozenset(
            int(geom_id)
            for geom_id in range(model.ngeom)
            if int(model.geom_bodyid[geom_id]) in body_ids
        )

    @staticmethod
    def _free_joint_addresses(*, mujoco_module: Any, model: Any, body_id: int) -> tuple[int, int]:
        joint_type_enum = getattr(mujoco_module, "mjtJoint", None)
        free_joint_type = getattr(joint_type_enum, "mjJNT_FREE", None) if joint_type_enum is not None else None
        if free_joint_type is None:
            return -1, -1
        joint_start = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        for joint_id in range(joint_start, joint_start + joint_count):
            if int(model.jnt_type[joint_id]) == int(free_joint_type):
                return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])
        return -1, -1


__all__ = ["CourseBuoy", "CourseBuoyRuntime"]
