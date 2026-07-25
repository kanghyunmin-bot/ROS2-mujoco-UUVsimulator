"""Sampling and output lifecycle helpers for ALT_HOLD diagnostics."""

from __future__ import annotations

from dataclasses import asdict

from althold_diagnostics_contract import Snapshot
from althold_diagnostics_output import finite, print_summary, write_csv, write_plot


def _sample(self) -> None:
    if self.done:
        return
    t_s = self._elapsed()
    row = Snapshot(**asdict(self.state))
    row.t_s = t_s
    self.rows.append(row)
    if t_s >= self.duration_s:
        self._finish()


def _finish(self) -> None:
    if self.done:
        return
    self.done = True
    write_csv(self.csv_path, self.rows)
    if self.plot:
        write_plot(self.plot_path, self.rows)
    print_summary(self.rows, csv_path=self.csv_path, plot_path=self.plot_path, plot_enabled=self.plot)


def _finite(values: list[float]) -> list[float]:
    return finite(values)
