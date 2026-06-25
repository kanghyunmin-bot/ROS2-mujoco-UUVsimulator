# ALT_HOLD Diagnostics Output Split

Date: 2026-06-07

Scope:

- `tools/althold_diagnostics_output.py`
- `tools/althold_diagnostics_csv.py`
- `tools/althold_diagnostics_series.py`
- `tools/althold_diagnostics_plot.py`
- `tools/althold_diagnostics_summary.py`

Intent:

- Keep the ALT_HOLD diagnostic output path small enough to audit.
- Preserve the existing compatibility import surface:
  `finite`, `print_summary`, `write_csv`, and `write_plot`.
- Avoid mixing CSV field ownership, plot rendering, terminal summary text, and
  numeric finite filtering in one branch-heavy module.

Verification:

- `PYTHONPATH=uuv_mujoco/current/tools python3 - <<'PY' ... PY`
  smoke-tested finite filtering, CSV output, optional plotting, and terminal
  summary output.
- `python3 -m compileall -q uuv_mujoco/current/tools/althold_diagnostics_*.py`
- `python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py`
- `git diff --check`

Contract notes:

- This split is behavior-neutral.  It does not change ArduSub parameters,
  controller-parity observation points, MuJoCo physics coefficients, PWM
  routing, or Bar30/IMU sensor equations.
- `matplotlib` remains optional; missing plotting support only skips the PNG
  output and does not block CSV/summary diagnostics.
