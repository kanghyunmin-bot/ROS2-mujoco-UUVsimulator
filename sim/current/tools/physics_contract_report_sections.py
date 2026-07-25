"""Compatibility exports for static physics contract report sections."""

from __future__ import annotations

from physics_contract_report_balance import (
    print_balance_section,
    print_neutral_sims_section,
    print_output_paths,
)
from physics_contract_report_body import print_mass_section
from physics_contract_report_hydro import print_hydrostatic_section, print_start_depth_section


__all__ = [
    "print_balance_section",
    "print_hydrostatic_section",
    "print_mass_section",
    "print_neutral_sims_section",
    "print_output_paths",
    "print_start_depth_section",
]
