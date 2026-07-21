"""Runner-level fluid contract setup."""

from __future__ import annotations

from dataclasses import dataclass

from sim.physics.fluid_contract import configure_fluid_model_contract


@dataclass(frozen=True)
class RunnerFluidContractSetup:
    fluid_model: str
    use_custom_hydrodynamics: bool


def create_runner_fluid_contract_setup(*, args, initial_setup) -> RunnerFluidContractSetup:
    fluid_model = str(args.fluid_model)
    use_custom_hydrodynamics = configure_fluid_model_contract(
        model=initial_setup.model,
        fluid_model=fluid_model,
        scene_path=args.scene,
        scene_fluid_density=initial_setup.scene_fluid_density,
        scene_fluid_viscosity=initial_setup.scene_fluid_viscosity,
    )
    return RunnerFluidContractSetup(
        fluid_model=fluid_model,
        use_custom_hydrodynamics=bool(use_custom_hydrodynamics),
    )


__all__ = ["RunnerFluidContractSetup", "create_runner_fluid_contract_setup"]
