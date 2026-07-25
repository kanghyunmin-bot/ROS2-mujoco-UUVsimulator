"""Context assembly for static MuJoCo physics contract audits."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from physics_contract_model import apply_runtime_body_contract, body_subtree_mass, mujoco
from physics_contract_profile_runtime import resolve_physics_contract_profile


@dataclass
class PhysicsContractAuditContext:
    scene: Path
    profile_path: Path
    output_dir: Path
    model: Any
    data: Any
    rho: float
    sim_profile: dict[str, Any]
    hydro_cfg: Any
    base_id: int
    body_contract: Any
    vehicle_mass: float
    gravity: float
    neutral_volume: float
    buoyancy_scale: float


def resolve_audit_paths(args: Any) -> tuple[Path, Path, Path]:
    scene = Path(args.scene).expanduser().resolve()
    profile_path = Path(args.profile_file).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return scene, profile_path, output_dir


def build_physics_contract_audit_context(args: Any) -> PhysicsContractAuditContext:
    scene, profile_path, output_dir = resolve_audit_paths(args)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rho = float(model.opt.density if model.opt.density > 0 else 1000.0)
    sim_profile, hydro_cfg = resolve_physics_contract_profile(profile_path, args.profile, args, rho)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        raise SystemExit("base_link body not found")

    body_contract = apply_runtime_body_contract(model, data, base_id, hydro_cfg, sim_profile)
    vehicle_mass = body_subtree_mass(model, base_id)
    if vehicle_mass <= 1.0e-9:
        vehicle_mass = float(model.body_mass[base_id])
    gravity = abs(float(model.opt.gravity[2]))
    neutral_volume = float(vehicle_mass / max(rho, 1.0e-9))
    if hydro_cfg.displaced_volume is not None and hydro_cfg.displaced_volume > 0.0:
        neutral_volume = float(hydro_cfg.displaced_volume)
    buoyancy_scale = float(hydro_cfg.buoyancy_scale)

    return PhysicsContractAuditContext(
        scene=scene,
        profile_path=profile_path,
        output_dir=output_dir,
        model=model,
        data=data,
        rho=rho,
        sim_profile=sim_profile,
        hydro_cfg=hydro_cfg,
        base_id=base_id,
        body_contract=body_contract,
        vehicle_mass=vehicle_mass,
        gravity=gravity,
        neutral_volume=neutral_volume,
        buoyancy_scale=buoyancy_scale,
    )


__all__ = [
    "PhysicsContractAuditContext",
    "build_physics_contract_audit_context",
    "resolve_audit_paths",
]
