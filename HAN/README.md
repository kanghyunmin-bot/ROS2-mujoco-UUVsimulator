# HAN Research Package

This package is for offline hydrodynamic coefficient estimation and validation.
It is not a live runtime patch surface.

Ownership:

- `data`: replay, CFD, and geometry-derived datasets.
- `features`: matrix/basis construction and normalization.
- `models`: HAN model definitions and frozen artifacts.
- `training`: optimization loops.
- `calibration`: projection, bounds, SVD/matrix gates, and residual fitting.
- `cfd`: CFD adapters and geometry metadata.
- `inference`: export of frozen profiles for runtime consumption.

Runtime rule: live MuJoCo code may load only a frozen profile that has passed a
matrix gate and plant replay validation.  It must not import training loops.
