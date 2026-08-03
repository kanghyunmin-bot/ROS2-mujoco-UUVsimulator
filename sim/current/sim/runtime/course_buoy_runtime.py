"""Competition-course buoy force and magnetic release runtime."""

from __future__ import annotations

from collections.abc import Collection
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
    cob_site_id: int
    eq_id: int
    collector_eq_id: int
    flex_line_bottom_eq_id: int
    flex_line_top_eq_id: int
    free_qposadr: int
    free_dofadr: int
    geom_ids: frozenset[int]
    geom_collision_bits: tuple[tuple[int, int, int], ...]
    flex_line_geom_collision_bits: tuple[tuple[int, int, int], ...]
    projection_geom_ids: tuple[int, ...]
    has_magnet: bool
    # The model mass, gravity, water surface and configured net lift are fixed
    # for the lifetime of one runtime.  Cache the two scalar results that were
    # previously rebuilt (including NumPy scalar dispatch) several times per
    # buoy and physics step.
    cached_body_weight_n: float = 0.0
    cached_surface_target_center_z: float = 0.0
    release_radius_m: float = 0.055
    detached: bool = False
    surface_on_waterline: bool = False
    collisions_suppressed: bool = False
    flex_line_collisions_suppressed: bool = False
    netted_gate_collision_enabled: bool | None = None
    netting: bool = False
    netting_time_s: float = -1.0
    netted: bool = False
    netted_time_s: float = -1.0
    net_slot_index: int = -1
    net_reverse_released: bool = False
    net_reverse_release_time_s: float = -1.0
    net_score_released: bool = False
    net_score_release_time_s: float = -1.0
    release_time_s: float = -1.0
    last_vehicle_contact_time_s: float = -1.0
    contact_release_start_time_s: float = -1.0
    contact_release_peak_n: float = 0.0
    contact_break_start_time_s: float = -1.0
    contact_break_peak_n: float = 0.0
    surface_escape_active: bool = False
    last_runtime_wrench: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    cached_linear_velocity_world: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64), repr=False
    )
    cached_angular_velocity_world: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64), repr=False
    )
    velocity_cache_generation: int = -1


@dataclass
class CourseBuoyRuntime:
    """Apply per-buoy float force and a breakable magnetic hold."""

    mujoco_module: Any
    model: Any
    data: Any
    buoys: list[CourseBuoy]
    vehicle_geom_ids: frozenset[int]
    vehicle_release_probe_geom_ids: tuple[int, ...]
    vehicle_root_body_id: int
    collector_body_id: int
    buoyancy_n: float
    vertical_damping_nspm: float
    water_linear_drag_nspm: float
    water_quadratic_drag_nspm2: float
    water_angular_drag_nmsprad: float
    float_half_height_m: float
    surface_spring_npm: float
    surface_capture_band_m: float
    vehicle_contact_grace_s: float
    release_stabilize_s: float
    release_collision_grace_s: float
    release_max_down_speed_mps: float
    release_max_up_speed_mps: float
    release_max_horizontal_speed_mps: float
    release_max_angular_speed_rps: float
    release_contact_clearance_m: float
    break_force_n: float
    contact_break_hold_s: float
    contact_release_hold_s: float
    proximity_release_enable: bool
    proximity_release_clearance_m: float
    magnet_stiffness_npm: float
    magnet_damping_nspm: float
    collector_net_enable: bool
    collector_net_window_x_m: float
    collector_net_window_y_m: float
    collector_net_window_z_m: float
    collector_net_hold_local: tuple[float, float, float]
    collector_net_score_probe_local: tuple[float, float, float]
    collector_net_stiffness_npm: float
    collector_net_damping_nspm: float
    collector_net_max_force_n: float
    collector_net_surface_gate_z_m: float
    collector_net_reverse_release_enable: bool
    collector_net_reverse_speed_mps: float
    collector_net_score_release_enable: bool
    collector_net_score_radius_m: float
    collector_net_score_z_window_m: float
    collector_net_score_settle_s: float
    collector_net_score_release_phase_gate: bool
    collector_net_score_release_status_path: Path | None
    surface_max_angular_speed_rps: float
    score_zone_a: tuple[float, float, float]
    score_zone_b: tuple[float, float, float]
    water_surface_z: float
    water_current_world: np.ndarray
    update_period_s: float
    force_update_period_s: float
    force_near_field_m: float
    track_csv_path: Path | None
    track_interval_s: float
    log: Callable[[str], None]
    _last_track_time_s: float = -1.0
    _track_header_written: bool = False
    _next_update_time_s: float = -1.0
    _next_force_update_time_by_body: dict[int, float] = field(default_factory=dict)
    _score_release_phase_check_time_s: float = -1.0
    _score_release_phase_allowed: bool = False
    _score_release_phase_state: str = ""
    _score_release_zone_target: np.ndarray | None = None
    _buoy_body_by_geom: dict[int, int] = field(default_factory=dict)
    _release_probe_geom_id_set: frozenset[int] = field(default_factory=frozenset)
    _contact_force_scratch: np.ndarray = field(
        default_factory=lambda: np.zeros(6, dtype=np.float64), repr=False
    )
    _velocity_cache_generation: int = 0
    _velocity_cache_active: bool = False

    @classmethod
    def from_model(
        cls,
        *,
        mujoco_module: Any,
        model: Any,
        data: Any,
        water_surface_z: float,
        water_current_world: np.ndarray | None = None,
        env_float: Callable[[str, float], float],
        env_flag: Callable[[str, bool], bool],
        log: Callable[[str], None],
    ) -> "CourseBuoyRuntime":
        if not env_flag("UUV_COURSE_BUOYS_ENABLE", True):
            return cls._empty(
                model=model,
                data=data,
                water_surface_z=water_surface_z,
                water_current_world=water_current_world,
                log=log,
            )

        obj_body = mujoco_module.mjtObj.mjOBJ_BODY
        obj_geom = mujoco_module.mjtObj.mjOBJ_GEOM
        obj_site = mujoco_module.mjtObj.mjOBJ_SITE
        obj_equality = mujoco_module.mjtObj.mjOBJ_EQUALITY
        vehicle_geom_ids = cls._body_subtree_geom_ids(
            mujoco_module=mujoco_module,
            model=model,
            root_body_name="base_link",
        )
        vehicle_release_probe_geom_ids = tuple(
            int(geom_id)
            for geom_id in range(model.ngeom)
            if (mujoco_module.mj_id2name(model, obj_geom, geom_id) or "").startswith(
                ("mission_port_rake_", "mission_starboard_rake_")
            )
        )
        vehicle_root_body_id = int(mujoco_module.mj_name2id(model, obj_body, "base_link"))
        collector_body_id = int(mujoco_module.mj_name2id(model, obj_body, "front_open_buoy_collector"))
        buoys: list[CourseBuoy] = []
        for body_id in range(model.nbody):
            body_name = mujoco_module.mj_id2name(model, obj_body, body_id) or ""
            if not body_name.startswith("course_buoy_") or not body_name.endswith("_float"):
                continue
            prefix = body_name.removesuffix("_float")
            attach_site_id = mujoco_module.mj_name2id(model, obj_site, f"{prefix}_attach_site")
            magnet_site_id = mujoco_module.mj_name2id(model, obj_site, f"{prefix}_magnet_site")
            cob_site_id = mujoco_module.mj_name2id(model, obj_site, f"{prefix}_cob_site")
            eq_id = mujoco_module.mj_name2id(model, obj_equality, f"{prefix}_magnet_weld")
            collector_eq_id = mujoco_module.mj_name2id(model, obj_equality, f"{prefix}_collector_weld")
            flex_line_top_eq_id = mujoco_module.mj_name2id(
                model,
                obj_equality,
                f"{prefix}_flex_line_top_connect",
            )
            flex_line_bottom_eq_id = mujoco_module.mj_name2id(
                model,
                obj_equality,
                f"{prefix}_flex_line_bottom_connect",
            )
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
            geom_collision_bits = tuple(
                (
                    int(geom_id),
                    int(model.geom_contype[int(geom_id)]),
                    int(model.geom_conaffinity[int(geom_id)]),
                )
                for geom_id in sorted(geom_ids)
            )
            flex_line_geom_collision_bits = tuple(
                (
                    int(geom_id),
                    int(model.geom_contype[int(geom_id)]),
                    int(model.geom_conaffinity[int(geom_id)]),
                )
                for geom_id in range(model.ngeom)
                if (
                    mujoco_module.mj_id2name(model, obj_geom, int(geom_id)) or ""
                ).startswith(f"{prefix}_flex_line_")
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
            release_radius_m = cls._release_radius_from_geoms(
                mujoco_module=mujoco_module,
                model=model,
                geom_ids=geom_ids,
            )
            buoys.append(
                CourseBuoy(
                    name=prefix,
                    body_id=int(body_id),
                    attach_site_id=int(attach_site_id),
                    magnet_site_id=int(magnet_site_id),
                    cob_site_id=int(cob_site_id),
                    eq_id=int(eq_id),
                    collector_eq_id=int(collector_eq_id),
                    flex_line_bottom_eq_id=int(flex_line_bottom_eq_id),
                    flex_line_top_eq_id=int(flex_line_top_eq_id),
                    free_qposadr=free_qposadr,
                    free_dofadr=free_dofadr,
                    geom_ids=geom_ids,
                    geom_collision_bits=geom_collision_bits,
                    flex_line_geom_collision_bits=flex_line_geom_collision_bits,
                    projection_geom_ids=projection_geom_ids,
                    has_magnet=has_magnet,
                    release_radius_m=release_radius_m,
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
            vehicle_release_probe_geom_ids=vehicle_release_probe_geom_ids,
            vehicle_root_body_id=vehicle_root_body_id,
            collector_body_id=collector_body_id,
            # Net lift at full immersion. MuJoCo gravity remains active, so the
            # applied water force also includes the body's weight.
            buoyancy_n=float(env_float("UUV_COURSE_BUOY_BUOYANCY_N", 1.0)),
            vertical_damping_nspm=float(env_float("UUV_COURSE_BUOY_VERTICAL_DAMPING_NSPM", 0.65)),
            water_linear_drag_nspm=float(env_float("UUV_COURSE_BUOY_WATER_LINEAR_DRAG_NSPM", 0.25)),
            # 0.5*rho*Cd*A for the 110 mm float is about 4.5 N/(m/s)^2
            # (rho=1000 kg/m^3, Cd~=0.95, A=pi*0.055^2).  The former 0.45
            # value was an order of magnitude too small and needed direct
            # qvel clamps to hide the resulting ~1.2 m/s terminal rise.
            water_quadratic_drag_nspm2=float(
                env_float("UUV_COURSE_BUOY_WATER_QUADRATIC_DRAG_NSPM2", 4.5)
            ),
            water_angular_drag_nmsprad=float(env_float("UUV_COURSE_BUOY_WATER_ANGULAR_DRAG_NMSPRAD", 0.0008)),
            float_half_height_m=float(env_float("UUV_COURSE_BUOY_FLOAT_HALF_HEIGHT_M", 0.085)),
            surface_spring_npm=float(env_float("UUV_COURSE_BUOY_SURFACE_SPRING_NPM", 2.0)),
            surface_capture_band_m=float(env_float("UUV_COURSE_BUOY_SURFACE_CAPTURE_BAND_M", 0.25)),
            vehicle_contact_grace_s=float(env_float("UUV_COURSE_BUOY_VEHICLE_CONTACT_GRACE_S", 0.45)),
            release_stabilize_s=float(env_float("UUV_COURSE_BUOY_RELEASE_STABILIZE_S", 0.45)),
            # Equality deactivation does not require a collision ghost window.
            # Keep the compatibility knob, but default it to zero and preserve
            # float/PVC contacts continuously through magnet release.
            release_collision_grace_s=float(env_float("UUV_COURSE_BUOY_RELEASE_COLLISION_GRACE_S", 0.0)),
            release_max_down_speed_mps=float(env_float("UUV_COURSE_BUOY_RELEASE_MAX_DOWN_SPEED_MPS", 0.05)),
            release_max_up_speed_mps=float(env_float("UUV_COURSE_BUOY_RELEASE_MAX_UP_SPEED_MPS", 0.45)),
            release_max_horizontal_speed_mps=float(env_float("UUV_COURSE_BUOY_RELEASE_MAX_HORIZONTAL_SPEED_MPS", 0.35)),
            release_max_angular_speed_rps=float(env_float("UUV_COURSE_BUOY_RELEASE_MAX_ANGULAR_SPEED_RPS", 1.20)),
            release_contact_clearance_m=float(env_float("UUV_COURSE_BUOY_RELEASE_CONTACT_CLEARANCE_M", 0.0)),
            break_force_n=float(env_float("UUV_COURSE_BUOY_MAGNET_BREAK_N", 15.0)),
            contact_break_hold_s=float(env_float("UUV_COURSE_BUOY_CONTACT_BREAK_HOLD_S", 0.04)),
            contact_release_hold_s=float(env_float("UUV_COURSE_BUOY_CONTACT_RELEASE_HOLD_S", 0.0)),
            proximity_release_enable=bool(env_flag("UUV_COURSE_BUOY_PROXIMITY_RELEASE_ENABLE", False)),
            proximity_release_clearance_m=float(env_float("UUV_COURSE_BUOY_PROXIMITY_RELEASE_CLEARANCE_M", 0.055)),
            magnet_stiffness_npm=float(env_float("UUV_COURSE_BUOY_MAGNET_STIFFNESS_NPM", 1200.0)),
            magnet_damping_nspm=float(env_float("UUV_COURSE_BUOY_MAGNET_DAMPING_NSPM", 8.0)),
            collector_net_enable=bool(env_flag("UUV_COURSE_BUOY_COLLECTOR_NET_ENABLE", True)),
            collector_net_window_x_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_WINDOW_X_M", 0.30)),
            collector_net_window_y_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_WINDOW_Y_M", 0.27)),
            collector_net_window_z_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_WINDOW_Z_M", 0.20)),
            collector_net_hold_local=(
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_HOLD_X_M", -0.011)),
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_HOLD_Y_M", 0.0)),
                # CoB-centered slot: keeps the 170 mm PVC tail above the
                # collector floor while retaining roof clearance.
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_HOLD_Z_M", 0.405)),
            ),
            collector_net_score_probe_local=(
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_PROBE_X_M", -0.135)),
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_PROBE_Y_M", 0.0)),
                float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_PROBE_Z_M", 0.501)),
            ),
            collector_net_stiffness_npm=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_STIFFNESS_NPM", 4.0)),
            collector_net_damping_nspm=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_DAMPING_NSPM", 1.2)),
            collector_net_max_force_n=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_MAX_FORCE_N", 3.0)),
            collector_net_surface_gate_z_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_NET_SURFACE_GATE_Z_M", 0.12)),
            collector_net_reverse_release_enable=bool(
                env_flag("UUV_COURSE_BUOY_COLLECTOR_REVERSE_RELEASE_ENABLE", True)
            ),
            collector_net_reverse_speed_mps=float(
                env_float("UUV_COURSE_BUOY_COLLECTOR_REVERSE_SPEED_MPS", 0.08)
            ),
            collector_net_score_release_enable=bool(env_flag("UUV_COURSE_BUOY_COLLECTOR_SCORE_RELEASE_ENABLE", True)),
            collector_net_score_radius_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_RADIUS_M", 0.45)),
            collector_net_score_z_window_m=float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_Z_WINDOW_M", 1.25)),
            collector_net_score_settle_s=float(env_float("UUV_COURSE_BUOY_COLLECTOR_SCORE_SETTLE_S", 4.0)),
            collector_net_score_release_phase_gate=bool(
                env_flag("UUV_COURSE_BUOY_COLLECTOR_SCORE_RELEASE_REQUIRE_MISSION_PHASE", True)
            ),
            # Release authorization is supplied live over ROS2 by the mission
            # node.  The field remains for constructor compatibility only.
            collector_net_score_release_status_path=None,
            surface_max_angular_speed_rps=float(env_float("UUV_COURSE_BUOY_SURFACE_MAX_ANGULAR_SPEED_RPS", 0.25)),
            score_zone_a=(
                float(env_float("UUV_COURSE_A_SCORE_X_M", -6.8)),
                float(env_float("UUV_COURSE_A_SCORE_Y_M", 0.0)),
                float(env_float("UUV_COURSE_A_SCORE_Z_M", -0.3)),
            ),
            score_zone_b=(
                float(env_float("UUV_COURSE_B_SCORE_X_M", 6.8)),
                float(env_float("UUV_COURSE_B_SCORE_Y_M", 0.0)),
                float(env_float("UUV_COURSE_B_SCORE_Z_M", -0.3)),
            ),
            water_surface_z=float(water_surface_z),
            water_current_world=cls._normalized_water_current(water_current_world),
            update_period_s=cls._update_period_s(env_float),
            force_update_period_s=cls._force_update_period_s(env_float),
            force_near_field_m=max(
                0.25,
                float(env_float("UUV_COURSE_BUOY_FORCE_NEAR_FIELD_M", 1.5)),
            ),
            track_csv_path=cls._track_csv_path(env_flag("UUV_COURSE_BUOY_TRACK_CSV_ENABLE", True)),
            track_interval_s=float(env_float("UUV_COURSE_BUOY_TRACK_CSV_INTERVAL_S", 0.25)),
            log=log,
        )
        runtime._buoy_body_by_geom = {
            int(geom_id): int(buoy.body_id)
            for buoy in runtime.buoys
            for geom_id in buoy.geom_ids
        }
        runtime._release_probe_geom_id_set = frozenset(runtime.vehicle_release_probe_geom_ids)
        # Category 4 is a runtime-only NETTED gate.  An MjModel may be reused
        # by acceptance tests or a soft runtime restart after a previous
        # capture, so clear that transient bit before deriving FREE state.
        # The cache then makes the normal per-step FREE/NETTED checks no-ops.
        for buoy in runtime.buoys:
            runtime._set_netted_gate_collision(buoy, enabled=False)
        runtime._initialize_static_float_contract()
        if buoys:
            log(
                "[course] buoys enabled: "
                f"count={len(buoys)}, full_immersion_net_lift={runtime.buoyancy_n:.3f}N, "
                f"magnet_break={runtime.break_force_n:.3f}N, "
                f"contact_break_hold={runtime.contact_break_hold_s:.3f}s, "
                f"contact_release_hold={runtime.contact_release_hold_s:.3f}s, "
                f"probe_proximity={'on' if runtime.proximity_release_enable else 'off'}"
            )
            runtime._log_float_contract()
            runtime._log_track_contract()
            runtime._log_force_update_contract()
        return runtime

    @classmethod
    def _empty(
        cls,
        *,
        model: Any,
        data: Any,
        water_surface_z: float,
        water_current_world: np.ndarray | None = None,
        log: Callable[[str], None],
    ) -> "CourseBuoyRuntime":
        return cls(
            mujoco_module=None,
            model=model,
            data=data,
            buoys=[],
            vehicle_geom_ids=frozenset(),
            vehicle_release_probe_geom_ids=(),
            vehicle_root_body_id=-1,
            collector_body_id=-1,
            buoyancy_n=0.0,
            vertical_damping_nspm=0.0,
            water_linear_drag_nspm=0.0,
            water_quadratic_drag_nspm2=0.0,
            water_angular_drag_nmsprad=0.0,
            float_half_height_m=0.085,
            surface_spring_npm=2.0,
            surface_capture_band_m=0.25,
            vehicle_contact_grace_s=0.45,
            release_stabilize_s=0.0,
            release_collision_grace_s=0.0,
            release_max_down_speed_mps=0.0,
            release_max_up_speed_mps=0.0,
            release_max_horizontal_speed_mps=0.0,
            release_max_angular_speed_rps=0.0,
            release_contact_clearance_m=0.0,
            break_force_n=0.0,
            contact_break_hold_s=0.0,
            contact_release_hold_s=0.0,
            proximity_release_enable=False,
            proximity_release_clearance_m=0.0,
            magnet_stiffness_npm=0.0,
            magnet_damping_nspm=0.0,
            collector_net_enable=False,
            collector_net_window_x_m=0.0,
            collector_net_window_y_m=0.0,
            collector_net_window_z_m=0.0,
            collector_net_hold_local=(0.0, 0.0, 0.0),
            collector_net_score_probe_local=(0.0, 0.0, 0.0),
            collector_net_stiffness_npm=0.0,
            collector_net_damping_nspm=0.0,
            collector_net_max_force_n=0.0,
            collector_net_surface_gate_z_m=0.0,
            collector_net_reverse_release_enable=False,
            collector_net_reverse_speed_mps=0.0,
            collector_net_score_release_enable=False,
            collector_net_score_radius_m=0.0,
            collector_net_score_z_window_m=0.0,
            collector_net_score_settle_s=0.0,
            collector_net_score_release_phase_gate=False,
            collector_net_score_release_status_path=None,
            surface_max_angular_speed_rps=0.0,
            score_zone_a=(0.0, 0.0, 0.0),
            score_zone_b=(0.0, 0.0, 0.0),
            water_surface_z=float(water_surface_z),
            water_current_world=cls._normalized_water_current(water_current_world),
            update_period_s=0.0,
            force_update_period_s=0.0,
            force_near_field_m=1.5,
            track_csv_path=None,
            track_interval_s=0.25,
            log=log,
        )

    def apply(self, dt: float) -> None:
        if not self.buoys:
            return

        # Contact/release decisions and collector transitions must run at the
        # physics rate. A rake can cross a 13 mm PVC stem between two 10 Hz
        # mission updates, and the net entrance force must be refreshed on the
        # same cadence as the 10 g float hydrodynamics.
        logic_due = self._update_due()
        now_s = float(getattr(self.data, "time", 0.0))
        (
            contacted_buoy_bodies,
            rake_contacted_buoy_bodies,
            rake_contact_force_by_body,
        ) = self._contact_snapshot()

        self._velocity_cache_generation += 1
        self._velocity_cache_active = True
        try:
            for buoy in self.buoys:
                if not self._force_refresh_due(
                    buoy,
                    now_s=now_s,
                    contacted=buoy.body_id in contacted_buoy_bodies,
                    rake_contacted=buoy.body_id in rake_contacted_buoy_bodies,
                ):
                    # xfrc_applied is a persistent MuJoCo control input.  The
                    # fast competition profile intentionally holds the last
                    # runtime-owned force between far-field refreshes.  Exact
                    # mode has a zero period and always follows the old path.
                    continue
                self._clear_persisted_runtime_wrench(buoy)
                vehicle_contact = buoy.body_id in contacted_buoy_bodies
                was_detached = buoy.detached
                rake_contact = buoy.body_id in rake_contacted_buoy_bodies
                release_contact = rake_contact or self._has_release_probe_proximity(buoy)
                if release_contact:
                    buoy.last_vehicle_contact_time_s = float(getattr(self.data, "time", 0.0))
                else:
                    self._reset_contact_release_sample(buoy)
                if buoy.has_magnet and not buoy.detached:
                    self._release_if_contact_or_break_force(
                        buoy,
                        vehicle_contact=release_contact,
                        contact_force_n=rake_contact_force_by_body.get(buoy.body_id, 0.0),
                    )
                if buoy.detached and was_detached:
                    self._restore_released_buoy_collisions_after_grace(buoy)

                # _clear_persisted_runtime_wrench leaves this owned scratch buffer
                # at zero. Reuse it instead of allocating a fifth six-axis array
                # for every buoy and physics step; contribution order is unchanged.
                wrench = buoy.last_runtime_wrench
                wrench += self._float_buoyancy_wrench(buoy, vehicle_contact=vehicle_contact)
                wrench += self._water_drag_wrench(buoy, vehicle_contact=vehicle_contact, dt=dt)
                if buoy.detached and was_detached:
                    wrench += self._stabilize_released_buoy(
                        buoy,
                        vehicle_contact=vehicle_contact,
                        dt=dt,
                    )
                wrench += self._collector_net_wrench(buoy)
                if buoy.has_magnet and not buoy.detached:
                    wrench += self._magnet_hold_wrench(buoy, dt)

                self.data.xfrc_applied[buoy.body_id, :] += wrench
                self._apply_surface_float_guard(buoy, vehicle_contact=vehicle_contact)
        finally:
            self._velocity_cache_active = False
        if logic_due:
            self._write_tracking_sample()

    def status_by_name(self) -> dict[str, dict[str, object]]:
        """Return runtime-only buoy state keyed by both prefix and body name."""

        now_s = float(getattr(self.data, "time", 0.0))
        rows: dict[str, dict[str, object]] = {}
        for buoy in self.buoys:
            capture_state = (
                "SCORE_RELEASED"
                if buoy.net_score_released
                else "NETTED"
                if buoy.netted
                else "NETTING"
                if buoy.netting
                else "FREE"
            )
            row = {
                "collector_net_enabled": bool(self.collector_net_enable),
                "capture_state": capture_state,
                "netting": bool(buoy.netting),
                "netting_time_s": float(buoy.netting_time_s),
                "netted": bool(buoy.netted),
                "netted_time_s": float(buoy.netted_time_s),
                "net_reverse_released": bool(buoy.net_reverse_released),
                "net_reverse_release_time_s": float(buoy.net_reverse_release_time_s),
                "collector_eq_active": bool(
                    buoy.collector_eq_id >= 0 and bool(self.data.eq_active[buoy.collector_eq_id])
                ),
                "net_score_released": bool(buoy.net_score_released),
                "net_score_release_time_s": float(buoy.net_score_release_time_s),
                "release_time_s": float(buoy.release_time_s),
                "runtime_time_s": now_s,
            }
            rows[buoy.name] = row
            rows[f"{buoy.name}_float"] = row
        return rows

    def _clear_persisted_runtime_wrench(self, buoy: CourseBuoy) -> None:
        last = buoy.last_runtime_wrench
        current = self.data.xfrc_applied[buoy.body_id, :]
        # Six scalar comparisons are materially cheaper here than creating
        # NumPy active/sign/removable masks for all 25 buoys every 5 ms. Keep
        # the exact former removal rule so an independently applied external
        # force is never cleared with the runtime-owned wrench.
        for axis in range(6):
            last_value = float(last[axis])
            if abs(last_value) > 1.0e-9:
                current_value = float(current[axis])
                same_direction = (
                    (current_value > 0.0 and last_value > 0.0)
                    or (current_value < 0.0 and last_value < 0.0)
                )
                still_present = abs(current_value) >= 0.5 * abs(last_value)
                if same_direction and still_present:
                    current[axis] = current_value - last_value
            last[axis] = 0.0

    def _float_buoyancy_wrench(self, buoy: CourseBuoy, *, vehicle_contact: bool) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        if self.buoyancy_n <= 0.0:
            return wrench

        center_z = float(self._buoy_center_world(buoy)[2])
        if not self._touches_float_waterline(buoy, center_z):
            return wrench

        velocity_z = float(self._buoy_linear_velocity(buoy)[2])
        target_z = self._surface_target_center_z(buoy)
        neutral_upthrust_n = self._body_weight_n(buoy)
        # Keep the tethered magnet load at the original reserve lift so the
        # compliant attachment can tilt under contact. Once physically
        # detached, use the larger free-float reserve needed to hold the body
        # at the collector waterline. The release ramp below prevents a force
        # step at the transition.
        reserve_buoyancy_n = self.buoyancy_n if buoy.detached else min(self.buoyancy_n, 0.98)
        buoyancy_cap_n = reserve_buoyancy_n * self._release_buoyancy_scale(buoy)
        if vehicle_contact and center_z < target_z:
            wrench[2] = neutral_upthrust_n + buoyancy_cap_n
            self._add_buoyancy_moment(buoy, wrench)
            return wrench

        depth_error_m = target_z - center_z
        net_lift_n = (self.surface_spring_npm * depth_error_m) - (self.vertical_damping_nspm * velocity_z)
        upthrust_n = neutral_upthrust_n + self._clamp_scalar(net_lift_n, 0.0, buoyancy_cap_n)
        wrench[2] = upthrust_n
        self._add_buoyancy_moment(buoy, wrench)
        return wrench

    def _release_buoyancy_scale(self, buoy: CourseBuoy) -> float:
        if not buoy.detached or buoy.release_time_s < 0.0 or self.release_stabilize_s <= 0.0:
            return 1.0
        elapsed_s = float(getattr(self.data, "time", 0.0)) - float(buoy.release_time_s)
        if elapsed_s < 0.0 or elapsed_s >= self.release_stabilize_s:
            return 1.0
        progress = elapsed_s / max(self.release_stabilize_s, 1.0e-9)
        return 0.35 + 0.65 * self._clamp_scalar(progress, 0.0, 1.0)

    def _water_drag_wrench(self, buoy: CourseBuoy, *, vehicle_contact: bool, dt: float) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        center_z = float(self._buoy_center_world(buoy)[2])
        if not self._touches_float_waterline(buoy, center_z):
            return wrench

        # xfrc_applied is a world-frame force, so drag must use the same
        # world-frame water-relative velocity as the vehicle fluid contract.
        # Without this subtraction, a configured current moved the vehicle
        # but left free/captured course buoys in still water.
        velocity = self._buoy_linear_velocity(buoy) - self.water_current_world
        if vehicle_contact:
            velocity = velocity.copy()
            velocity[2] = 0.0
        speed = float(np.linalg.norm(velocity))
        drag_force = np.zeros(3, dtype=np.float64)
        if self.water_linear_drag_nspm > 0.0:
            drag_force -= self.water_linear_drag_nspm * velocity
        if self.water_quadratic_drag_nspm2 > 0.0 and speed > 0.0:
            # Projected wetted area falls rapidly as the float exits the
            # water. At the static waterline this 1 N/10 g contract is only
            # about 9% immersed; applying full submerged drag there made a
            # free float stick to the descending collector roof instead of
            # sliding naturally around it.
            immersion_fraction = self._clamp_scalar(
                (self.water_surface_z + self.float_half_height_m - center_z)
                / max(2.0 * self.float_half_height_m, 1.0e-9),
                0.0,
                1.0,
            )
            drag_force -= (
                self.water_quadratic_drag_nspm2
                * immersion_fraction
                * speed
                * velocity
            )
        if speed > 0.0:
            mass_kg = max(float(self.model.body_mass[buoy.body_id]), 1.0e-6)
            # Hydrodynamic wrenches are refreshed every physics step even when
            # mission-state logic is throttled, so the impulse limit must use
            # the physics step rather than the slower state-update period.  A
            # rising float also receives up to ``buoyancy_n`` in the opposite
            # direction, so include only that force's projection along the
            # relative velocity. This lets quadratic drag balance the 1 N
            # reserve lift without allowing a horizontal drag impulse to flip
            # the velocity in one step.
            force_hold_s = max(float(dt), 1.0e-6)
            projected_lift_n = 0.0
            if buoy.detached:
                projected_lift_n = self.buoyancy_n * float(velocity[2]) / speed
            max_drag_n = max(
                0.0,
                0.8 * mass_kg * speed / force_hold_s + projected_lift_n,
            )
            drag_n = float(np.linalg.norm(drag_force))
            if max_drag_n <= 0.0:
                drag_force[:] = 0.0
            elif drag_n > max_drag_n:
                drag_force *= max_drag_n / drag_n
        wrench[0:3] += drag_force

        angular_velocity = self._buoy_angular_velocity(buoy)
        if self.water_angular_drag_nmsprad > 0.0:
            angular_drag = -self.water_angular_drag_nmsprad * angular_velocity
            angular_speed = float(np.linalg.norm(angular_velocity))
            angular_drag_n = float(np.linalg.norm(angular_drag))
            if angular_speed > 0.0 and angular_drag_n > 0.0:
                inertia = np.asarray(self.model.body_inertia[buoy.body_id], dtype=np.float64)
                min_inertia = max(float(np.min(inertia)), 1.0e-9)
                max_angular_drag_nm = (
                    0.8 * min_inertia * angular_speed / max(float(dt), 1.0e-6)
                )
                if angular_drag_n > max_angular_drag_nm:
                    angular_drag *= max_angular_drag_nm / angular_drag_n
            wrench[3:6] += angular_drag
        return wrench

    def _collector_net_wrench(self, buoy: CourseBuoy) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        if (
            not self.collector_net_enable
            or not buoy.detached
            or buoy.free_qposadr < 0
            or buoy.free_dofadr < 0
            or self.vehicle_root_body_id < 0
        ):
            return wrench

        now_s = float(getattr(self.data, "time", 0.0))
        if buoy.net_score_released:
            return self._score_settle_wrench(buoy, now_s)

        buoy_pos = self._buoy_center_world(buoy)
        local = self._base_local_from_world(buoy_pos)
        if buoy.net_reverse_released:
            if self._collector_reverse_exit_complete(local, buoy):
                buoy.net_reverse_released = False
                buoy.net_slot_index = -1
                self.log(f"[course] buoy cleared open net mouth: {buoy.name}")
            return wrench
        if (
            not buoy.netting
            and not buoy.netted
            and self._collector_net_ready_for_capture(buoy, buoy_pos)
            and self._inside_collector_net_window(local, buoy)
        ):
            buoy.netting = True
            buoy.netting_time_s = now_s
            buoy.net_slot_index = self._allocate_collector_net_slot()
            buoy.net_reverse_released = False
            buoy.net_reverse_release_time_s = -1.0
            self._restore_buoy_collisions(buoy)
            self._suppress_flex_line_collisions(buoy)
            wrench += self._stabilize_surface_orientation(buoy)
            self.log(f"[course] buoy entering net: {buoy.name}")

        if not buoy.netting and not buoy.netted:
            return wrench

        # DIST3's bounded spring-damper guides only an already-entering float
        # to its assigned interior slot. The force cap prevents the collector
        # from becoming a long-range or through-wall capture mechanism.
        if buoy.netting or buoy.collector_eq_id < 0:
            target = self._collector_net_target_world(buoy)
            force = self.collector_net_stiffness_npm * (target - buoy_pos)
            force += self.collector_net_damping_nspm * (
                self._body_linear_velocity(self.vehicle_root_body_id)
                - self._buoy_linear_velocity(buoy)
            )
            wrench[0:3] = self._limited_force(force, self.collector_net_max_force_n)

        if buoy.netting and self._collector_net_ready_to_close(local, buoy):
            buoy.netting = False
            buoy.netted = True
            buoy.netted_time_s = now_s
            if buoy.collector_eq_id >= 0:
                self._activate_collector_weld(buoy)
                wrench[:] = 0.0
            # The soft weld carries the buoy but is not a rigid wall. The roof
            # is permanently physical; close the category-4 front flap for
            # every NETTED buoy, including models that provide a weld.
            self._set_netted_gate_collision(buoy, enabled=True)
            self.log(f"[course] buoy netted: {buoy.name}")

        if not buoy.netted:
            return wrench

        # NETTED is a physical pocket invariant: the weld, permanent roof, and
        # gated front flap work together. Reassert the category so a recreated
        # runtime or a temporarily reactivated weld cannot leave a captured
        # buoy protected by the soft equality alone.
        self._set_netted_gate_collision(buoy, enabled=True)

        # The Git reference collector is a rigid pocket with an open +X mouth.
        # Keep the soft coupling during forward/turn/depth motion, but drop it
        # without modifying qpos/qvel when the vehicle backs away. The buoy can
        # then leave through the same physical opening under contact and drag.
        if self._collector_reverse_release_requested():
            buoy.netted = False
            buoy.netting = False
            buoy.net_reverse_released = True
            buoy.net_reverse_release_time_s = now_s
            self._deactivate_collector_weld(buoy)
            self._set_netted_gate_collision(buoy, enabled=False)
            self.log(f"[course] buoy reverse-released through open net mouth: {buoy.name}")
            return wrench

        # Forward/turn/depth carry remains softly coupled until a deliberate
        # reverse exit or the score release gate allows release.
        if buoy.collector_eq_id >= 0 and not bool(self.data.eq_active[buoy.collector_eq_id]):
            self._activate_collector_weld(buoy)
            wrench[:] = 0.0

        if (
            self.collector_net_score_release_enable
            and self._score_release_phase_allowed_now(now_s)
            and self._collector_inside_score_zone()
        ):
            buoy.netted = False
            buoy.netting = False
            buoy.net_reverse_released = False
            buoy.net_score_released = True
            buoy.net_score_release_time_s = now_s
            self._deactivate_collector_weld(buoy)
            self._set_netted_gate_collision(buoy, enabled=False)
            self._release_netted_buoy_into_score_zone(buoy)
            self.log(f"[course] buoy net released in score zone: {buoy.name}")
            return self._score_settle_wrench(buoy, now_s)

        return wrench

    def _collector_net_ready_for_capture(self, buoy: CourseBuoy, buoy_pos: np.ndarray) -> bool:
        gate = max(0.0, float(self.collector_net_surface_gate_z_m))
        if gate <= 0.0:
            return True
        target_z = self._surface_target_center_z(buoy)
        hold_world = self._world_from_base_local(np.asarray(self.collector_net_hold_local, dtype=np.float64))
        if abs(float(hold_world[2] - target_z)) > gate:
            return False
        if float(buoy_pos[2]) < target_z - gate:
            return False
        return True

    def _set_netted_gate_collision(self, buoy: CourseBuoy, *, enabled: bool) -> None:
        requested = bool(enabled)
        if buoy.netted_gate_collision_enabled is requested:
            return
        netted_gate_category = 4
        original = {
            int(geom_id): (int(contype), int(conaffinity))
            for geom_id, contype, conaffinity in buoy.geom_collision_bits
        }
        for geom_id in buoy.geom_ids:
            geom_id = int(geom_id)
            original_contype, original_conaffinity = original.get(
                geom_id,
                (
                    int(self.model.geom_contype[geom_id]),
                    int(self.model.geom_conaffinity[geom_id]),
                ),
            )
            # Decorative/fluid proxy geoms must remain non-colliding. Only the
            # physical float/PVC geoms receive the NETTED-only gate category.
            if original_contype == 0 and (original_conaffinity & ~netted_gate_category) == 0:
                self.model.geom_conaffinity[geom_id] = 0
                continue
            # Runtime instances can be recreated against an already-mutated
            # MjModel. Strip the temporary gate category before applying the
            # requested state so OFF always reopens the +X mouth.
            base_conaffinity = original_conaffinity & ~netted_gate_category
            if requested:
                self.model.geom_conaffinity[geom_id] = base_conaffinity | netted_gate_category
            else:
                self.model.geom_conaffinity[geom_id] = base_conaffinity
        buoy.netted_gate_collision_enabled = requested

    def _activate_collector_weld(self, buoy: CourseBuoy) -> None:
        if buoy.collector_eq_id < 0:
            return
        slot = self._collector_net_slot_local(buoy)
        collector_local = np.asarray(self.model.body_pos[self.collector_body_id], dtype=np.float64)
        # Slots describe the center of buoyancy while a weld relpose describes
        # the buoy body frame. Converting with each buoy's local CoB supports
        # both upright underwater floats and horizontal surface floats.
        cob_local = (
            np.asarray(self.model.site_pos[buoy.cob_site_id], dtype=np.float64)
            if buoy.cob_site_id >= 0
            else np.zeros(3, dtype=np.float64)
        )
        relative = slot - collector_local - cob_local
        # Weld equality data stores anchor[0:3], relpose position[3:6],
        # relpose quaternion[6:10], and torque scale[10].
        self.model.eq_data[buoy.collector_eq_id, 3:6] = relative
        self.model.eq_data[buoy.collector_eq_id, 6:10] = [1.0, 0.0, 0.0, 0.0]
        self.data.eq_active[buoy.collector_eq_id] = 1

    def _deactivate_collector_weld(self, buoy: CourseBuoy) -> None:
        if buoy.collector_eq_id >= 0:
            self.data.eq_active[buoy.collector_eq_id] = 0

    def _collector_reverse_release_requested(self) -> bool:
        if not self.collector_net_reverse_release_enable or self.vehicle_root_body_id < 0:
            return False
        rot = np.asarray(self.data.xmat[self.vehicle_root_body_id], dtype=np.float64).reshape(3, 3)
        velocity_world = self._body_linear_velocity(self.vehicle_root_body_id)
        forward_speed_mps = float((rot.T @ velocity_world)[0])
        threshold = max(0.0, float(self.collector_net_reverse_speed_mps))
        return forward_speed_mps <= -threshold

    @staticmethod
    def _collector_reverse_exit_complete(local: np.ndarray, buoy: CourseBuoy) -> bool:
        # +X is the open mouth. Clear the re-capture latch only after the full
        # float radius is outside the 315 mm front frame plane.
        mouth_clear_x = 0.315 + max(0.0, float(buoy.release_radius_m)) + 0.015
        return bool(
            float(local[0]) >= mouth_clear_x
            or abs(float(local[1])) >= 0.39
            or float(local[2]) <= -0.08
            or float(local[2]) >= 0.60
        )

    def _collector_net_ready_to_close(self, local: np.ndarray, buoy: CourseBuoy) -> bool:
        slot = self._collector_net_slot_local(buoy)
        if float(np.linalg.norm(local - slot)) > 0.10:
            return False
        # Center clearances use the float radius, not the full tilted capsule
        # extent: the slot force keeps a newly entered float nearly upright.
        return (
            -0.25 <= float(local[0]) <= 0.23
            and abs(float(local[1])) <= 0.25
            and 0.22 <= float(local[2]) <= 0.55
        )

    def _inside_collector_net_window(self, local: np.ndarray, buoy: CourseBuoy) -> bool:
        hold = np.asarray(self.collector_net_hold_local, dtype=np.float64)
        target_z = self._surface_target_center_z(buoy)
        base_z = float(self.data.xpos[self.vehicle_root_body_id, 2])
        center = hold.copy()
        center[2] = target_z - base_z
        window_x = max(0.0, float(self.collector_net_window_x_m))
        # Arm the per-buoy net flap only while the float crosses the forward
        # mouth. A float vertically above the basket must remain free when the
        # vehicle dives beneath it.
        mouth_inner_x = float(center[0]) + 0.45 * window_x
        return (
            mouth_inner_x <= float(local[0]) <= float(center[0]) + window_x
            and abs(float(local[1] - center[1])) <= max(0.0, float(self.collector_net_window_y_m))
            and abs(float(local[2] - center[2])) <= max(0.0, float(self.collector_net_window_z_m))
        )

    def _collector_net_target_world(self, buoy: CourseBuoy) -> np.ndarray:
        return self._world_from_base_local(self._collector_net_slot_local(buoy))

    def _allocate_collector_net_slot(self) -> int:
        used = {item.net_slot_index for item in self.buoys if item.net_slot_index >= 0}
        for index in range(13):
            if index not in used:
                return index
        return len(used) % 13

    def _collector_net_slot_local(self, buoy: CourseBuoy) -> np.ndarray:
        index = max(0, int(buoy.net_slot_index)) % 13
        # Fill back-to-front while keeping the center of the forward mouth
        # clear for the next float. Within each row, use the inner lanes first.
        slot_xy = (
            (-0.12, -0.11), (-0.12, 0.11), (-0.12, -0.22), (-0.12, 0.22),
            (0.00, -0.11), (0.00, 0.11), (0.00, -0.22), (0.00, 0.22),
            (0.12, -0.11), (0.12, 0.11), (0.12, -0.22), (0.12, 0.22),
            (-0.12, 0.00),
        )
        slot_x, slot_y = slot_xy[index]
        center = np.asarray(self.collector_net_hold_local, dtype=np.float64)
        return center + np.array(
            [slot_x, slot_y, 0.0],
            dtype=np.float64,
        )

    def _collector_inside_score_zone(self) -> bool:
        probe = self._world_from_base_local(np.asarray(self.collector_net_score_probe_local, dtype=np.float64))
        if self._score_release_zone_target is not None:
            zones = (self._score_release_zone_target,)
        elif self.collector_net_score_release_phase_gate:
            return False
        else:
            zones = (self.score_zone_a, self.score_zone_b)
        for zone in zones:
            zone_pos = np.asarray(zone, dtype=np.float64)
            d = probe - zone_pos
            if (
                float(np.hypot(d[0], d[1])) <= max(0.0, float(self.collector_net_score_radius_m))
                and abs(float(d[2])) <= max(0.0, float(self.collector_net_score_z_window_m))
            ):
                return True
        return False

    def _score_release_phase_allowed_now(self, now_s: float) -> bool:
        del now_s
        if not self.collector_net_score_release_phase_gate:
            return True
        return bool(self._score_release_phase_allowed)

    def set_score_release_contract(self, state: str, score_zone_xyz: Any) -> bool:
        """Update the live ROS mission release authorization.

        A release remains subject to the physical score-zone radius and height
        checks.  Invalid or non-release messages close the gate immediately.
        """
        state = str(state or "").strip().upper()
        self._score_release_phase_state = state
        try:
            zone_target = np.asarray(score_zone_xyz, dtype=np.float64).reshape(3)
        except (TypeError, ValueError):
            zone_target = None
        valid_zone = zone_target is not None and bool(np.all(np.isfinite(zone_target)))
        allowed_state = state in {"RELEASE", "SCORE_RELEASE"}
        if not valid_zone:
            self._score_release_zone_target = None
        else:
            self._score_release_zone_target = zone_target
        self._score_release_phase_allowed = bool(allowed_state and valid_zone)
        return self._score_release_phase_allowed

    def _release_netted_buoy_into_score_zone(self, buoy: CourseBuoy) -> None:
        # Releasing an equality must preserve the exact generalized state.
        # The previous implementation injected a fraction of the vehicle
        # velocity and a vertical kick here, producing a visible jump at the
        # release boundary.  Subsequent buoyancy, drag and the bounded score
        # settle wrench provide the physical motion; qpos/qvel stay untouched.
        _ = buoy

    def _score_settle_wrench(self, buoy: CourseBuoy, now_s: float) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        settle_until = float(buoy.net_score_release_time_s) + max(0.0, float(self.collector_net_score_settle_s))
        if buoy.net_score_release_time_s < 0.0 or now_s > settle_until:
            return wrench
        target = self._nearest_score_zone_target(buoy)
        if target is None:
            return wrench
        pos = self._buoy_center_world(buoy)
        force = 0.55 * self.collector_net_stiffness_npm * (target - pos)
        force[2] = 0.0
        wrench[0:3] = self._limited_force(force, 0.55 * self.collector_net_max_force_n)
        return wrench

    def _nearest_score_zone_target(self, buoy: CourseBuoy) -> np.ndarray | None:
        pos = self._buoy_center_world(buoy)
        zones = (
            [self._score_release_zone_target.copy()]
            if self._score_release_zone_target is not None
            else [np.asarray(self.score_zone_a, dtype=np.float64), np.asarray(self.score_zone_b, dtype=np.float64)]
        )
        if not zones:
            return None
        target = min(zones, key=lambda zone: float(np.hypot(pos[0] - zone[0], pos[1] - zone[1]))).copy()
        index = max(0, int(buoy.net_slot_index)) % 16
        target[0] += (-0.165, -0.055, 0.055, 0.165)[index // 4]
        target[1] += (-0.165, -0.055, 0.055, 0.165)[index % 4]
        target[2] = self._surface_target_center_z(buoy)
        return target

    def _world_from_base_local(self, local_pos: np.ndarray) -> np.ndarray:
        rot = np.array(self.data.xmat[self.vehicle_root_body_id], dtype=np.float64).reshape(3, 3)
        return np.array(self.data.xpos[self.vehicle_root_body_id], dtype=np.float64) + rot @ local_pos

    def _base_local_from_world(self, world_pos: np.ndarray) -> np.ndarray:
        rot = np.array(self.data.xmat[self.vehicle_root_body_id], dtype=np.float64).reshape(3, 3)
        return rot.T @ (world_pos - np.array(self.data.xpos[self.vehicle_root_body_id], dtype=np.float64))

    @staticmethod
    def _limited_force(force: np.ndarray, max_force_n: float) -> np.ndarray:
        limit = max(0.0, float(max_force_n))
        if limit <= 0.0:
            return np.zeros(3, dtype=np.float64)
        norm = float(np.linalg.norm(force))
        if norm > limit:
            return force * (limit / max(norm, 1.0e-9))
        return force

    def _touches_float_waterline(self, buoy: CourseBuoy, center_z: float) -> bool:
        return float(center_z) <= self.water_surface_z + self.float_half_height_m

    def _surface_target_center_z(self, buoy: CourseBuoy) -> float:
        return float(buoy.cached_surface_target_center_z)

    def _body_weight_n(self, buoy: CourseBuoy) -> float:
        return float(buoy.cached_body_weight_n)

    def _initialize_static_float_contract(self) -> None:
        """Cache model/config-only float scalars without changing the force law."""

        gravity_z = float(self.model.opt.gravity[2])
        for buoy in self.buoys:
            weight_n = float(self.model.body_mass[buoy.body_id]) * abs(gravity_z)
            buoy.cached_body_weight_n = weight_n
            if self.buoyancy_n <= 0.0:
                target_z = self.water_surface_z - self.float_half_height_m
            else:
                full_upthrust_n = weight_n + self.buoyancy_n
                equilibrium_fraction = self._clamp_scalar(
                    weight_n / full_upthrust_n,
                    0.0,
                    1.0,
                )
                target_z = (
                    self.water_surface_z
                    + self.float_half_height_m
                    - 2.0 * self.float_half_height_m * equilibrium_fraction
                )
            buoy.cached_surface_target_center_z = float(target_z)

    @staticmethod
    def _clamp_scalar(value: float, lower: float, upper: float) -> float:
        """Scalar equivalent of ``np.clip(value, lower, upper)`` for finite inputs."""

        return min(max(float(value), float(lower)), float(upper))

    def _buoy_center_world(self, buoy: CourseBuoy) -> np.ndarray:
        if buoy.cob_site_id >= 0:
            return np.array(self.data.site_xpos[buoy.cob_site_id], dtype=np.float64)
        return np.array(self.data.xipos[buoy.body_id], dtype=np.float64)

    def _add_buoyancy_moment(self, buoy: CourseBuoy, wrench: np.ndarray) -> None:
        if buoy.cob_site_id < 0:
            return
        center_of_mass = self.data.xipos[buoy.body_id]
        center_of_buoyancy = self.data.site_xpos[buoy.cob_site_id]
        rx = float(center_of_buoyancy[0]) - float(center_of_mass[0])
        ry = float(center_of_buoyancy[1]) - float(center_of_mass[1])
        rz = float(center_of_buoyancy[2]) - float(center_of_mass[2])
        fx = float(wrench[0])
        fy = float(wrench[1])
        fz = float(wrench[2])
        wrench[3] += ry * fz - rz * fy
        wrench[4] += rz * fx - rx * fz
        wrench[5] += rx * fy - ry * fx

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
        starts = [float(self._buoy_center_world(buoy)[2]) for buoy in self.buoys]
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
            f"collector_net={'on' if self.collector_net_enable else 'off'}"
            f"(window={self.collector_net_window_x_m:.2f}/{self.collector_net_window_y_m:.2f}/{self.collector_net_window_z_m:.2f}m,"
            f" max_force={self.collector_net_max_force_n:.2f}N,"
            f" surface_gate_z={self.collector_net_surface_gate_z_m:.2f}m,"
            f" score_phase_gate={'on' if self.collector_net_score_release_phase_gate else 'off'}), "
            f"samples=[{samples}]"
        )

    def _log_track_contract(self) -> None:
        if self.track_csv_path is None:
            self.log("[course] buoy live tracking disabled")
            return
        update_hz = (1.0 / self.update_period_s) if self.update_period_s > 0.0 else 0.0
        self.log(
            "[course] buoy live tracking: "
            f"path={self.track_csv_path}, interval={max(self.track_interval_s, 0.0):.3f}s, "
            f"update_hz={'physics' if update_hz <= 0.0 else f'{update_hz:.1f}'}"
        )

    def _log_force_update_contract(self) -> None:
        period_s = float(self.force_update_period_s)
        if period_s <= 0.0:
            self.log("[course] buoy force cadence: exact physics-step refresh")
            return
        self.log(
            "[course] buoy force cadence: "
            f"far_field={1.0 / period_s:.1f}Hz(sim), "
            f"near_field=physics(within {self.force_near_field_m:.2f}m/contact/net)"
        )

    @staticmethod
    def _update_period_s(env_float: Callable[[str, float], float]) -> float:
        update_hz = float(env_float("UUV_COURSE_BUOY_UPDATE_HZ", 0.0))
        if update_hz <= 0.0:
            return 0.0
        return 1.0 / max(1.0, min(500.0, update_hz))

    @staticmethod
    def _force_update_period_s(env_float: Callable[[str, float], float]) -> float:
        update_hz = float(env_float("UUV_COURSE_BUOY_FORCE_UPDATE_HZ", 0.0))
        if update_hz <= 0.0:
            return 0.0
        return 1.0 / max(10.0, min(500.0, update_hz))

    def _force_refresh_due(
        self,
        buoy: CourseBuoy,
        *,
        now_s: float,
        contacted: bool,
        rake_contacted: bool,
    ) -> bool:
        period_s = float(self.force_update_period_s)
        if period_s <= 0.0:
            return True

        urgent = bool(
            contacted
            or rake_contacted
            or buoy.netting
            or buoy.netted
            or buoy.net_reverse_released
            or buoy.net_score_released
            or buoy.surface_escape_active
        )
        if not urgent and self.vehicle_root_body_id >= 0:
            vehicle_pos = self.data.xpos[self.vehicle_root_body_id]
            buoy_pos = self.data.xpos[buoy.body_id]
            dx = float(vehicle_pos[0]) - float(buoy_pos[0])
            dy = float(vehicle_pos[1]) - float(buoy_pos[1])
            dz = float(vehicle_pos[2]) - float(buoy_pos[2])
            near_m = float(self.force_near_field_m)
            urgent = dx * dx + dy * dy + dz * dz <= near_m * near_m

        body_id = int(buoy.body_id)
        if urgent:
            self._next_force_update_time_by_body[body_id] = float(now_s) + period_s
            return True

        next_due = self._next_force_update_time_by_body.get(body_id, -1.0)
        if next_due < 0.0 or float(now_s) + 1.0e-9 >= float(next_due):
            self._next_force_update_time_by_body[body_id] = float(now_s) + period_s
            return True
        return False

    def _update_due(self) -> bool:
        period_s = float(self.update_period_s)
        if period_s <= 0.0:
            return True
        now_s = float(getattr(self.data, "time", 0.0))
        if self._next_update_time_s < 0.0:
            self._next_update_time_s = now_s
        if now_s + 1.0e-9 < self._next_update_time_s:
            return False
        self._next_update_time_s = now_s + period_s
        return True

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
                    pos = self._buoy_center_world(buoy)
                    file.write(
                        f"{now_s:.3f},{buoy.name},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                        f"{velocity_z:.4f},{self._surface_target_center_z(buoy):.4f},"
                        f"{int(buoy.detached)},{int(buoy.surface_on_waterline)},{force_z:.4f}\n"
                    )
        except OSError as exc:
            self.log(f"[course] buoy live tracking disabled after write error: {exc}")
            self.track_csv_path = None

    def _stabilize_released_buoy(
        self,
        buoy: CourseBuoy,
        *,
        vehicle_contact: bool,
        dt: float | None = None,
    ) -> np.ndarray:
        """Return a continuous post-release damping wrench.

        Equality release is a state transition, not an impulse.  In
        particular this routine must never rewrite qpos, qvel, or qacc.  The
        speed limits are implemented as excess-speed damping and therefore
        enter MuJoCo through the same force integration as buoyancy and drag.
        """

        wrench = np.zeros(6, dtype=np.float64)
        if buoy.free_qposadr < 0 or buoy.free_dofadr < 0:
            return wrench
        now_s = float(getattr(self.data, "time", 0.0))
        target_z = self._surface_target_center_z(buoy)
        center_z = float(self._buoy_center_world(buoy)[2])
        in_release_window = (
            buoy.release_time_s >= 0.0
            and self.release_stabilize_s > 0.0
            and now_s <= buoy.release_time_s + self.release_stabilize_s
        )
        underwater = center_z < target_z - self.surface_capture_band_m
        recent_contact = self._recent_vehicle_contact(buoy)
        at_surface = center_z >= target_z - 0.005
        needs_release_damping = (
            in_release_window or underwater or vehicle_contact or recent_contact
        )
        if not (needs_release_damping or at_surface):
            return wrench

        step_s = max(
            float(dt) if dt is not None else float(getattr(self.model.opt, "timestep", 0.005)),
            1.0e-6,
        )
        mass_kg = max(float(self.model.body_mass[buoy.body_id]), 1.0e-6)
        velocity = self._buoy_linear_velocity(buoy)
        damping_nspm = max(0.0, float(self.water_linear_drag_nspm))

        if needs_release_damping and damping_nspm > 0.0:
            horizontal = velocity[0:2]
            horizontal_speed = float(np.linalg.norm(horizontal))
            max_horizontal = max(0.0, float(self.release_max_horizontal_speed_mps))
            if max_horizontal > 0.0 and horizontal_speed > max_horizontal:
                excess = horizontal * (1.0 - max_horizontal / horizontal_speed)
                wrench[0:2] -= damping_nspm * excess

            velocity_z = float(velocity[2])
            max_up = abs(float(self.release_max_up_speed_mps))
            max_down = abs(float(self.release_max_down_speed_mps))
            vertical_excess = 0.0
            if max_up > 0.0 and velocity_z > max_up:
                vertical_excess = velocity_z - max_up
            elif center_z < target_z and max_down > 0.0 and velocity_z < -max_down:
                vertical_excess = velocity_z + max_down
            wrench[2] -= damping_nspm * vertical_excess

            # The quadratic water drag may already consume up to 80% of the
            # momentum in one physics step. Bound this supplemental damping to
            # another 15%, leaving a numerical margin without a hidden clamp.
            force_norm = float(np.linalg.norm(wrench[0:3]))
            speed = float(np.linalg.norm(velocity))
            max_force_n = 0.15 * mass_kg * speed / step_s
            if force_norm > max_force_n > 0.0:
                wrench[0:3] *= max_force_n / force_norm

        if in_release_window and (vehicle_contact or recent_contact):
            wrench[0:3] += self._separate_released_buoy_from_vehicle(buoy)

        # A free surface float touching the sloped collector roof must roll
        # toward the open +X mouth as the vehicle dives. The old waterline
        # qvel clamp supplied an implicit infinite contact reaction and hid
        # this escape motion. Model the compliant net's small tangential
        # reaction explicitly instead; it is disabled once capture starts.
        if buoy.netting or buoy.netted:
            buoy.surface_escape_active = False
        if vehicle_contact and not buoy.netting and not buoy.netted:
            local = self._base_local_from_world(self._buoy_center_world(buoy))
            inside_roof_footprint = (
                abs(float(local[1])) <= self.collector_net_window_y_m + buoy.release_radius_m
                and float(local[2]) >= self.collector_net_hold_local[2] - self.float_half_height_m
            )
            if at_surface and inside_roof_footprint:
                buoy.surface_escape_active = True
            if buoy.surface_escape_active and inside_roof_footprint:
                base_rotation = np.asarray(
                    self.data.xmat[self.vehicle_root_body_id],
                    dtype=np.float64,
                ).reshape(3, 3)
                wrench[0:3] += 0.14 * base_rotation[:, 0]

        angular_limit = self.release_max_angular_speed_rps if needs_release_damping else 0.0
        if vehicle_contact:
            angular_limit = min(angular_limit, 0.50) if angular_limit > 0.0 else 0.50
        if at_surface:
            surface_limit = max(0.0, float(self.surface_max_angular_speed_rps))
            angular_limit = min(angular_limit, surface_limit) if angular_limit > 0.0 else surface_limit
        wrench += self._stabilize_surface_orientation(
            buoy,
            max_angular_speed_rps=angular_limit,
            dt=step_s,
        )
        return wrench

    def _apply_surface_float_guard(self, buoy: CourseBuoy, *, vehicle_contact: bool) -> None:
        """Track waterline state without rewriting generalized state."""

        if not buoy.detached or buoy.free_qposadr < 0 or buoy.free_dofadr < 0:
            return

        target_z = self._surface_target_center_z(buoy)
        center_z = float(self._buoy_center_world(buoy)[2])
        waterline_floor_z = target_z - 0.005
        if center_z < waterline_floor_z:
            buoy.surface_on_waterline = False
            return
        if vehicle_contact or self._recent_vehicle_contact(buoy):
            return

        if not buoy.surface_on_waterline:
            buoy.surface_on_waterline = True
        self._restore_buoy_collisions(buoy)

    def _stabilize_surface_orientation(
        self,
        buoy: CourseBuoy,
        *,
        max_angular_speed_rps: float | None = None,
        dt: float | None = None,
    ) -> np.ndarray:
        """Return bounded angular damping; never teleport orientation/rate."""

        wrench = np.zeros(6, dtype=np.float64)
        if buoy.free_qposadr < 0 or buoy.free_dofadr < 0:
            return wrench
        max_angular = max(
            0.0,
            float(
                self.surface_max_angular_speed_rps
                if max_angular_speed_rps is None
                else max_angular_speed_rps
            ),
        )
        angular = self._buoy_angular_velocity(buoy)
        angular_speed = float(np.linalg.norm(angular))
        if angular_speed <= max_angular or angular_speed <= 1.0e-9:
            return wrench
        excess = angular * (1.0 - max_angular / angular_speed)
        torque = -max(0.0, float(self.water_angular_drag_nmsprad)) * excess
        inertia = np.asarray(self.model.body_inertia[buoy.body_id], dtype=np.float64)
        min_inertia = max(float(np.min(inertia)), 1.0e-9)
        step_s = max(
            float(dt) if dt is not None else float(getattr(self.model.opt, "timestep", 0.005)),
            1.0e-6,
        )
        max_torque_nm = 0.15 * min_inertia * angular_speed / step_s
        torque_norm = float(np.linalg.norm(torque))
        if torque_norm > max_torque_nm > 0.0:
            torque *= max_torque_nm / torque_norm
        wrench[3:6] = torque
        return wrench

    def _recent_vehicle_contact(self, buoy: CourseBuoy) -> bool:
        if buoy.last_vehicle_contact_time_s < 0.0 or self.vehicle_contact_grace_s <= 0.0:
            return False
        elapsed_s = float(getattr(self.data, "time", 0.0)) - buoy.last_vehicle_contact_time_s
        return 0.0 <= elapsed_s <= self.vehicle_contact_grace_s

    def _vehicle_contacted_buoy_body_ids(self) -> set[int]:
        return self._contacted_buoy_body_ids(self.vehicle_geom_ids)

    def _release_probe_contacted_buoy_body_ids(self) -> set[int]:
        return self._contacted_buoy_body_ids(self._release_probe_geom_id_set)

    def _contact_snapshot(self) -> tuple[set[int], set[int], dict[int, float]]:
        """Classify all buoy/vehicle contacts and rake force in one ``ncon`` pass."""

        contacted: set[int] = set()
        rake_contacted: set[int] = set()
        rake_force_by_body: dict[int, float] = {}
        if not self._buoy_body_by_geom or not self.vehicle_geom_ids:
            return contacted, rake_contacted, rake_force_by_body

        contact_force = (
            getattr(self.mujoco_module, "mj_contactForce", None)
            if self.mujoco_module is not None
            else None
        )
        force = self._contact_force_scratch
        for contact_id in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            vehicle_geom = -1
            buoy_geom = -1
            if geom1 in self.vehicle_geom_ids and geom2 in self._buoy_body_by_geom:
                vehicle_geom, buoy_geom = geom1, geom2
            elif geom2 in self.vehicle_geom_ids and geom1 in self._buoy_body_by_geom:
                vehicle_geom, buoy_geom = geom2, geom1
            if buoy_geom < 0:
                continue

            body_id = int(self._buoy_body_by_geom[buoy_geom])
            contacted.add(body_id)
            if vehicle_geom not in self._release_probe_geom_id_set:
                continue
            rake_contacted.add(body_id)
            if contact_force is None:
                continue
            force[:] = 0.0
            contact_force(self.model, self.data, contact_id, force)
            # Keep the same norm primitive as the former per-buoy scanner so a
            # force exactly at the configured release threshold is classified
            # identically.  The slice is a view into the preallocated scratch.
            force_n = float(np.linalg.norm(force[0:3]))
            previous = rake_force_by_body.get(body_id, 0.0)
            if force_n > previous:
                rake_force_by_body[body_id] = force_n
        return contacted, rake_contacted, rake_force_by_body

    def _contacted_buoy_body_ids(self, vehicle_geom_ids: frozenset[int]) -> set[int]:
        if not self._buoy_body_by_geom or not vehicle_geom_ids:
            return set()
        contacted: set[int] = set()
        for contact_id in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            if geom1 in vehicle_geom_ids:
                body_id = self._buoy_body_by_geom.get(geom2)
                if body_id is not None:
                    contacted.add(body_id)
            if geom2 in vehicle_geom_ids:
                body_id = self._buoy_body_by_geom.get(geom1)
                if body_id is not None:
                    contacted.add(body_id)
        return contacted

    def _has_release_probe_proximity(self, buoy: CourseBuoy) -> bool:
        if (
            not self.proximity_release_enable
            or not self.vehicle_release_probe_geom_ids
            or buoy.body_id < 0
            or not buoy.geom_ids
        ):
            return False
        buoy_pos = self._buoy_center_world(buoy)
        buoy_radius = float(buoy.release_radius_m)
        clearance = max(0.0, float(self.proximity_release_clearance_m))
        for geom_id in self.vehicle_release_probe_geom_ids:
            center = self.data.geom_xpos[int(geom_id)]
            sizes = self.model.geom_size[int(geom_id)]
            probe_radius = float(sizes[0])
            probe_half_length = float(sizes[1])
            reach = probe_radius + probe_half_length + buoy_radius + clearance
            dx = float(buoy_pos[0]) - float(center[0])
            dy = float(buoy_pos[1]) - float(center[1])
            dz = float(buoy_pos[2]) - float(center[2])
            if dx * dx + dy * dy + dz * dz > reach * reach:
                continue
            surface_distance = self._point_capsule_surface_distance_m(int(geom_id), buoy_pos)
            if surface_distance <= buoy_radius + clearance:
                return True
        return False

    @staticmethod
    def _release_radius_from_geoms(*, mujoco_module: Any, model: Any, geom_ids: frozenset[int]) -> float:
        radius = 0.055
        for geom_id in geom_ids:
            name = mujoco_module.mj_id2name(model, mujoco_module.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
            if not name.endswith("_float_geom"):
                continue
            sizes = np.asarray(model.geom_size[int(geom_id)], dtype=np.float64).reshape(-1)
            if sizes.size:
                radius = max(radius, float(np.max(sizes[: min(3, sizes.size)])))
            break
        return radius

    def _point_capsule_surface_distance_m(self, geom_id: int, point: np.ndarray) -> float:
        center = self.data.geom_xpos[geom_id]
        sizes = self.model.geom_size[geom_id]
        radius = float(sizes[0])
        half_length = float(sizes[1])
        rx = float(point[0]) - float(center[0])
        ry = float(point[1]) - float(center[1])
        rz = float(point[2]) - float(center[2])
        if half_length <= 0.0:
            return float(np.sqrt(rx * rx + ry * ry + rz * rz)) - radius

        xmat = self.data.geom_xmat[geom_id]
        ax = float(xmat[2])
        ay = float(xmat[5])
        az = float(xmat[8])
        along = float(np.clip(rx * ax + ry * ay + rz * az, -half_length, half_length))
        dx = rx - ax * along
        dy = ry - ay * along
        dz = rz - az * along
        return float(np.sqrt(dx * dx + dy * dy + dz * dz)) - radius

    def _magnet_hold_wrench(self, buoy: CourseBuoy, dt: float) -> np.ndarray:
        wrench = np.zeros(6, dtype=np.float64)
        if buoy.eq_id >= 0:
            return wrench
        attach_pos = np.array(self.data.site_xpos[buoy.attach_site_id], dtype=np.float64)
        magnet_pos = np.array(self.data.site_xpos[buoy.magnet_site_id], dtype=np.float64)
        displacement = attach_pos - magnet_pos
        velocity = self._buoy_linear_velocity(buoy)
        damping = self.magnet_damping_nspm * velocity if dt > 0.0 else 0.0
        magnet_force = -(self.magnet_stiffness_npm * displacement + damping)
        force_norm = float(np.linalg.norm(magnet_force))
        if self.break_force_n > 0.0 and force_norm > self.break_force_n:
            magnet_force *= self.break_force_n / force_norm
        wrench[0:3] = magnet_force
        return wrench

    def _release_if_contact_or_break_force(
        self,
        buoy: CourseBuoy,
        *,
        vehicle_contact: bool,
        contact_force_n: float | None = None,
    ) -> None:
        contact_force_n = (
            self._contact_force_norm(buoy, vehicle_geom_ids=self._release_probe_geom_id_set)
            if contact_force_n is None and vehicle_contact
            else max(0.0, float(contact_force_n or 0.0))
        )
        if vehicle_contact and self._contact_release_sustained(buoy, contact_force_n):
            self._reset_contact_break_sample(buoy)
            if not buoy.detached:
                self._detach(
                    buoy,
                    reason=(
                        "rake_contact_immediate"
                        if self.contact_release_hold_s <= 0.0
                        else "rake_contact_force_hold"
                    ),
                    force_n=float(buoy.contact_release_peak_n),
                )
            self._reset_contact_release_sample(buoy)
            return
        # Physical rake contact owns this release path. The generic 15 N break
        # force remains available for non-rake loads only.
        self._release_if_break_force_exceeded(buoy, include_contact=False)

    def _release_if_break_force_exceeded(self, buoy: CourseBuoy, *, include_contact: bool) -> None:
        force_n, reason = self._release_force_sample(buoy, include_contact=include_contact)
        if force_n < self.break_force_n:
            self._reset_contact_break_sample(buoy)
            return
        if reason == "contact" and not self._contact_break_sustained(buoy, force_n):
            return
        if reason == "contact":
            force_n = max(force_n, float(buoy.contact_break_peak_n))
        self._reset_contact_break_sample(buoy)
        if not buoy.detached:
            self._detach(buoy, reason=reason, force_n=force_n)

    def _contact_release_sustained(self, buoy: CourseBuoy, force_n: float) -> bool:
        """Release immediately on rake contact, or retain the legacy debounce when configured."""
        hold_s = max(0.0, float(self.contact_release_hold_s))
        now_s = float(getattr(self.data, "time", 0.0))
        if buoy.contact_release_start_time_s < 0.0:
            buoy.contact_release_start_time_s = now_s
            buoy.contact_release_peak_n = max(0.0, float(force_n))
        else:
            buoy.contact_release_peak_n = max(float(buoy.contact_release_peak_n), float(force_n))
        if hold_s <= 0.0:
            return True
        force_ready = float(buoy.contact_release_peak_n) >= max(0.0, float(self.break_force_n))
        time_ready = now_s - buoy.contact_release_start_time_s >= hold_s
        return bool(force_ready and time_ready)

    @staticmethod
    def _reset_contact_release_sample(buoy: CourseBuoy) -> None:
        buoy.contact_release_start_time_s = -1.0
        buoy.contact_release_peak_n = 0.0

    def _contact_break_sustained(self, buoy: CourseBuoy, force_n: float) -> bool:
        hold_s = max(0.0, float(self.contact_break_hold_s))
        if hold_s <= 0.0:
            return True
        now_s = float(getattr(self.data, "time", 0.0))
        if buoy.contact_break_start_time_s < 0.0:
            buoy.contact_break_start_time_s = now_s
            buoy.contact_break_peak_n = float(force_n)
            return False
        buoy.contact_break_peak_n = max(float(buoy.contact_break_peak_n), float(force_n))
        if now_s - buoy.contact_break_start_time_s < hold_s:
            return False
        return True

    @staticmethod
    def _reset_contact_break_sample(buoy: CourseBuoy) -> None:
        buoy.contact_break_start_time_s = -1.0
        buoy.contact_break_peak_n = 0.0

    def _release_force_sample(self, buoy: CourseBuoy, *, include_contact: bool) -> tuple[float, str]:
        # Do not use equality/weld solver force here. It includes the magnet's
        # own support force and can exceed 15N immediately after startup.
        samples = [(self._external_force_norm(buoy), "external")]
        if include_contact:
            samples.append((self._contact_force_norm(buoy), "contact"))
        return max(samples, key=lambda item: item[0])

    def _external_force_norm(self, buoy: CourseBuoy) -> float:
        force = np.array(self.data.xfrc_applied[buoy.body_id, 0:3], dtype=np.float64)
        return float(np.linalg.norm(force))

    def _contact_force_norm(
        self,
        buoy: CourseBuoy,
        *,
        vehicle_geom_ids: Collection[int] | None = None,
    ) -> float:
        if self.mujoco_module is None or not buoy.geom_ids:
            return 0.0
        contact_force = getattr(self.mujoco_module, "mj_contactForce", None)
        if contact_force is None:
            return 0.0
        force = np.zeros(6, dtype=np.float64)
        max_force = 0.0
        contact_vehicle_geoms = self.vehicle_geom_ids if vehicle_geom_ids is None else vehicle_geom_ids
        for contact_id in range(int(getattr(self.data, "ncon", 0))):
            contact = self.data.contact[contact_id]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            buoy_hit = geom1 in buoy.geom_ids or geom2 in buoy.geom_ids
            vehicle_hit = geom1 in contact_vehicle_geoms or geom2 in contact_vehicle_geoms
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
        # Preserve float/PVC contact continuously. Disabling collision masks
        # for even a few physics steps lets the released buoy tunnel through a
        # rake, collector net, or hull. Only the released flex line is muted.
        self._restore_buoy_collisions(buoy)
        self._suppress_flex_line_collisions(buoy)
        if buoy.eq_id >= 0 and hasattr(self.data, "eq_active"):
            self.data.eq_active[buoy.eq_id] = 0
        if buoy.flex_line_top_eq_id >= 0 and hasattr(self.data, "eq_active"):
            self.data.eq_active[buoy.flex_line_top_eq_id] = 0
        # The rope releases from the float, not from the pool-floor anchor.
        # Releasing both ends leaves every cable segment as an unconstrained
        # free body and makes MuJoCo progressively slower after each capture.
        self._recompute_after_release(buoy)
        self._hide_surface_projection(buoy)
        self.log(f"[course] magnet detached: {buoy.name} reason={reason} force={force_n:.3f}N")

    def _separate_released_buoy_from_vehicle(self, buoy: CourseBuoy) -> np.ndarray:
        """Return the optional release-clearance force without moving the float."""

        force = np.zeros(3, dtype=np.float64)
        if buoy.free_qposadr < 0 or self.release_contact_clearance_m <= 0.0:
            return force
        root_body_id = self._vehicle_root_body_id()
        if root_body_id < 0:
            return force
        buoy_pos = self._buoy_center_world(buoy)
        vehicle_pos = np.array(self.data.xpos[root_body_id], dtype=np.float64)
        away = buoy_pos - vehicle_pos
        away[2] *= 0.35
        norm = float(np.linalg.norm(away))
        if norm <= 1.0e-9:
            away = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            away /= norm
        # A small up component keeps the released float clear of the lower body
        # while the buoyancy ramp takes over.
        away = away + np.array([0.0, 0.0, 0.35], dtype=np.float64)
        away /= max(float(np.linalg.norm(away)), 1.0e-9)
        separation_n = max(0.0, float(self.surface_spring_npm)) * float(
            self.release_contact_clearance_m
        )
        force[:] = away * min(separation_n, 0.25)
        return force

    def _reset_released_buoy_motion(self, buoy: CourseBuoy) -> None:
        """Compatibility no-op: release must preserve pose, velocity and acceleration."""

        del buoy

    def _recompute_after_release(self, buoy: CourseBuoy) -> None:
        forward = getattr(self.mujoco_module, "mj_forward", None) if self.mujoco_module is not None else None
        if forward is None:
            return
        try:
            forward(self.model, self.data)
        except Exception as exc:
            self.log(f"[course] release forward recompute skipped for {buoy.name}: {exc}")
            return

    def _vehicle_root_body_id(self) -> int:
        if self.mujoco_module is None:
            return -1
        return int(self.mujoco_module.mj_name2id(self.model, self.mujoco_module.mjtObj.mjOBJ_BODY, "base_link"))

    def _suppress_buoy_collisions(self, buoy: CourseBuoy) -> None:
        if buoy.collisions_suppressed:
            return
        for geom_id, contype, conaffinity in buoy.geom_collision_bits:
            if contype == 0 and conaffinity == 0:
                continue
            self.model.geom_contype[int(geom_id)] = 0
            self.model.geom_conaffinity[int(geom_id)] = 0
        buoy.collisions_suppressed = True
        buoy.netted_gate_collision_enabled = None

    def _suppress_flex_line_collisions(self, buoy: CourseBuoy) -> None:
        if buoy.flex_line_collisions_suppressed:
            return
        for geom_id, contype, conaffinity in buoy.flex_line_geom_collision_bits:
            if contype == 0 and conaffinity == 0:
                continue
            self.model.geom_contype[int(geom_id)] = 0
            self.model.geom_conaffinity[int(geom_id)] = 0
        buoy.flex_line_collisions_suppressed = True

    def _restore_buoy_collisions(self, buoy: CourseBuoy) -> None:
        if not buoy.collisions_suppressed:
            return
        for geom_id, contype, conaffinity in buoy.geom_collision_bits:
            self.model.geom_contype[int(geom_id)] = int(contype)
            self.model.geom_conaffinity[int(geom_id)] = int(conaffinity)
        buoy.collisions_suppressed = False
        buoy.netted_gate_collision_enabled = None

    def _restore_released_buoy_collisions_after_grace(self, buoy: CourseBuoy) -> None:
        """Repair any inherited suppressed mask without introducing ghost time.

        The fixed-line collision stays suppressed separately. Float and PVC
        collision masks remain physical from the equality-release step onward.
        """

        if not buoy.collisions_suppressed or buoy.release_time_s < 0.0:
            return
        self._restore_buoy_collisions(buoy)

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

    def _buoy_linear_velocity(self, buoy: CourseBuoy) -> np.ndarray:
        """Current world-frame CoM velocity from the free-joint state.

        ``data.cvel`` is produced during forward dynamics and can trail the
        just-integrated ``qvel`` by one sample at the normal apply/step call
        boundary. That delay is harmless for logging but destabilizes the
        stiff quadratic drag of a 10 g float. Free-joint translation is world
        framed; its angular part is body framed, so include the CoM offset.
        """

        generation = self._velocity_cache_generation
        if self._velocity_cache_active and buoy.velocity_cache_generation == generation:
            return buoy.cached_linear_velocity_world
        if buoy.free_dofadr < 0:
            velocity = self._body_linear_velocity(buoy.body_id)
            if self._velocity_cache_active:
                buoy.cached_linear_velocity_world[:] = velocity
                buoy.velocity_cache_generation = generation
                return buoy.cached_linear_velocity_world
            return velocity
        dof = buoy.free_dofadr
        angular_world = self._buoy_angular_velocity(buoy)
        origin_velocity = self.data.qvel[dof : dof + 3]
        center = self.data.xipos[buoy.body_id]
        origin = self.data.xpos[buoy.body_id]
        ox = float(center[0]) - float(origin[0])
        oy = float(center[1]) - float(origin[1])
        oz = float(center[2]) - float(origin[2])
        wx, wy, wz = float(angular_world[0]), float(angular_world[1]), float(angular_world[2])
        velocity = (
            buoy.cached_linear_velocity_world
            if self._velocity_cache_active
            else np.empty(3, dtype=np.float64)
        )
        velocity[0] = float(origin_velocity[0]) + (wy * oz - wz * oy)
        velocity[1] = float(origin_velocity[1]) + (wz * ox - wx * oz)
        velocity[2] = float(origin_velocity[2]) + (wx * oy - wy * ox)
        if self._velocity_cache_active:
            buoy.velocity_cache_generation = generation
        return velocity

    def _buoy_angular_velocity(self, buoy: CourseBuoy) -> np.ndarray:
        """Current world-frame angular velocity from the free-joint state."""

        generation = self._velocity_cache_generation
        if self._velocity_cache_active and buoy.velocity_cache_generation == generation:
            return buoy.cached_angular_velocity_world
        if buoy.free_dofadr < 0:
            angular = self._body_angular_velocity(buoy.body_id)
            if self._velocity_cache_active:
                buoy.cached_angular_velocity_world[:] = angular
                return buoy.cached_angular_velocity_world
            return angular
        angular_body = np.asarray(
            self.data.qvel[buoy.free_dofadr + 3 : buoy.free_dofadr + 6],
            dtype=np.float64,
        )
        rotation = np.asarray(self.data.xmat[buoy.body_id], dtype=np.float64).reshape(3, 3)
        if self._velocity_cache_active:
            angular = buoy.cached_angular_velocity_world
            np.dot(rotation, angular_body, out=angular)
            # Do not mark the shared generation valid until linear velocity is
            # also populated; a later linear request still needs its CoM term.
            return buoy.cached_angular_velocity_world
        return rotation @ angular_body

    @staticmethod
    def _normalized_water_current(value: np.ndarray | None) -> np.ndarray:
        if value is None:
            return np.zeros(3, dtype=np.float64)
        current = np.asarray(value, dtype=np.float64).reshape(-1)
        if current.size != 3 or not bool(np.all(np.isfinite(current))):
            raise ValueError("course buoy water_current_world must contain three finite values")
        return current.astype(np.float64, copy=True)

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
