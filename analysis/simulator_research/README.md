# UUV Simulation Research Workspace

This directory is the navigation layer for the v2.2 sim-to-real work.  It does
not own the raw data.  Large outputs stay in their original locations so old
commands and paper references remain reproducible.

## Directory Roles

| folder | role | current rule |
| --- | --- | --- |
| `00_current_contract` | active simulator/controller contract | Only accepted configuration, params, actuator and sensor contracts belong here. |
| `01_controller_parity` | ArduSub controller parity evidence | Real `/mavros/rc/out` vs SITL `SERVO_OUTPUT_RAW`; no JSON-servo-to-real comparison. |
| `02_plant_replay` | real RCOU -> MuJoCo sensor validation | Main plant identification gate before closed-loop tuning. |
| `03_han_cfd` | HAN/CFD coefficient identification | Candidate generation, priors, sensitivity, rejected/accepted profiles. |
| `04_paper_evidence` | figures/tables/commands for paper | Curated references only; each entry must point to raw output. |
| `90_legacy_before` | before/current-contract history | Preserved for chronology and ablation history, not current evidence. |

## Current Status

| layer | status | decision |
| --- | --- | --- |
| Source contract audit | `PASS=10`, `WARN=5`, `FAIL=0` against clean `ArduSub-4.1.2`. | Use `/research_workspace/00_current_contract/contract_source_audit.*` as current code-level contract evidence; do not commit the ArduPilot gitlink unless intentionally updating it. |
| Docker ArduSub build | Verified in Ubuntu Docker compose path. | Use Docker SITL as the supported ArduSub build/runtime path; ignore native macOS `fenv` build failure for parity work. |
| Controller parity | Mostly aligned after telemetry-to-telemetry comparison. | Keep using real RCOU/SERVO telemetry as controller output reference. |
| Plant replay input | Exact in latest audit: `maxdiff_us=0`, `mismatches=0`. | RCOU timing/input is no longer the primary root cause. |
| MuJoCo plant response | Still fails 99% target. | Continue plant model identification; do not hide with controller or PWM shims. |
| Fossen residual damping | Implemented behind zero default coefficients. | Available for candidates, not promoted by default. |
| CFD/HAN prior | Pipeline now supports residual quadratic terms. | Current CFD prior is paper evidence, not accepted final tuning. |

## Priority

1. **P0 Source Contract Audit**: keep ArduSub 4.1.2 source reference clean; verify JSON, Bar30, RCOU, RC override, IMU, DVL, and thruster contracts before tuning.
2. **P1 Current Baseline Reproduction**: regenerate plant replay baseline and verify `plant_input maxdiff_us=0`.
3. **P2 Plant Model Identification**: fit coupled yaw-sway and heave-pitch response using full90 real RCOU replay.
4. **P3 HAN/CFD Completion**: use CFD only as bounded prior, then validate every candidate through plant replay.
5. **P4 Paper Evidence**: promote only runs with commands, metrics, overlays, and raw output paths into `04_paper_evidence`.

## Artifact Catalog

The generated catalog is the source of truth for current vs legacy output
classification:

```bash
python3 /Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/catalog_outputs.py
```

Generated files:

- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/_catalog/artifact_catalog.md`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/_catalog/artifact_catalog.csv`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/_catalog/artifact_catalog.json`

## Non-Destructive Policy

- Do not move or delete raw output folders unless a separate migration manifest
  exists.
- Do not rewrite ArduPilot or change its submodule pointer.
- Do not promote a profile into `config/sim_profiles.json` unless a direct
  validation run beats baseline and the contract gate passes.
- Do not classify legacy runs as current evidence unless `FINDINGS.md` names
  the exact reason.
