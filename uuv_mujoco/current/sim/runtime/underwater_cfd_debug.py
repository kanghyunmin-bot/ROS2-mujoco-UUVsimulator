"""Debug logging for CFD dynamic hydrodynamic wrench runtime."""

from __future__ import annotations

import numpy as np


def maybe_log_cfd_dynamic_wrench(*, hyd, data, rel_lin_vel_body, cfd_force_body, submerged: float) -> None:
    if not hyd.cfd_dynamic_wrench_debug:
        return
    if float(data.time) - hyd.cfd_dynamic_wrench_last_log_sim_t["value"] < 2.0:
        return
    hyd.cfd_dynamic_wrench_last_log_sim_t["value"] = float(data.time)
    print(
        "[physics] CFD dynamic wrench update: "
        f"rel_body={np.array2string(rel_lin_vel_body, precision=3)}, "
        f"force_body={np.array2string(cfd_force_body, precision=3)}, "
        f"submerged={submerged:.3f}",
        flush=True,
    )


__all__ = ["maybe_log_cfd_dynamic_wrench"]
