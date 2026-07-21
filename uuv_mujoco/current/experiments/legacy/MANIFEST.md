# Legacy Manifest

Legacy material is retained for provenance, not for active runtime imports.

Rules:

- Do not import active code from this folder.
- Do not delete old evidence unless it has been copied into a named baseline or
  paper-evidence folder.
- When moving old outputs here, preserve the original folder name and add a
  note explaining why it is legacy.
- Prefer manifests over bulk copying when the source folder is very large.

Initial legacy candidates:

- historical `logs/althold_*` runs,
- historical `debug/physics_contract/*` sweeps,
- failed controller-parity runs with header-only or disarmed plant-side output,
- old HAN/CFD coefficient sweeps that did not pass the matrix gate.
