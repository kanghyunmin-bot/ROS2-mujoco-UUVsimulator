# Paper Evidence Index

This index lists the current evidence that is clean enough to cite or turn into
figures/tables.  It points to raw outputs rather than copying them.

## Core Evidence

| claim | source | paper use |
| --- | --- | --- |
| Literature references for linear-algebra plant identification are fixed with DOI/BibTeX keys. | `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/04_paper_evidence/REFERENCES_20260601.md` and `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/04_paper_evidence/references_20260601.bib` | References section and related work. |
| Controller output must be compared telemetry-to-telemetry. | `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/FINDINGS.md` | Methods: controller parity contract. |
| Plant replay input is exact in latest full90 audits. | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/fossen_residual_full90_20260601/baseline/plant_replay/sensor_overlay/plant_sensor_contract_audit.json` | Methods/validation: real RCOU input contract. |
| Fossen residual coefficient family was evaluated but not accepted as final. | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/fossen_residual_full90_20260601/fossen_residual_ranking.md` | Ablation: residual damping sensitivity. |
| Linear algebra identification gate now ranks parameter subsets before least squares. | `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/04_paper_evidence/LINEAR_ALGEBRA_IDENTIFICATION_REVIEW_20260601.md` | Methods: rank-revealing sensitivity and identifiability. |
| Fossen full90 sensitivity has low residual coverage despite acceptable condition number. | `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_case_matrix_search_fossen_all.md` | Ablation: why blind LS is rejected. |
| CFD prior improved aggregate score slightly but failed final gate. | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/validate_fossen_cfd_prior_full90_20260601/ranking.md` | Ablation: CFD prior limitation. |
| Latest plant overlay for CFD prior. | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/validate_fossen_cfd_prior_full90_20260601/plant_replay/sensor_overlay/plant_sensor_overlay.png` | Figure candidate, not final success figure. |
| Legacy/current split and artifact catalog. | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/_catalog/artifact_catalog.md` | Appendix/reproducibility. |

## Figure Candidates

1. Controller parity RCOU 6DOF decomposition overlay.
2. Plant replay sensor overlay for baseline vs CFD prior.
3. Fossen residual sensitivity ranking table.
4. CFD prior changed-coefficient table.
5. Contract pipeline diagram: real RCOU -> plant -> sensors.

## Not Final Evidence

Do not present `validate_fossen_cfd_prior_full90_20260601` as a solved
simulation.  It is evidence that CFD/HAN priors are connected and measurable,
but it still fails the full90 99% gate.
