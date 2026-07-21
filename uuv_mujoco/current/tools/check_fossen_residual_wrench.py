#!/usr/bin/env python3
"""Smoke-check Fossen residual damping wrench composition."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.fossen_residual_wrench import fossen_residual_wrench_body  # noqa: E402


def main() -> int:
    wrench = fossen_residual_wrench_body(
        np.array([2.0, -1.0, 0.5], dtype=float),
        np.array([0.1, -0.2, 0.3], dtype=float),
        linear={"x_u": 2.0, "y_v": 3.0, "ignored": 100.0},
        forward_speed={"z_u": 0.5, "n_r": 4.0},
        quadratic={"x_abs_u_u": 1.5, "m_abs_q_q": 2.0, "k_abs_p_r": 10.0},
    )
    expected = np.array(
        [
            -10.0,  # -2*u - 1.5*abs(u)*u
            3.0,  # -3*v
            -2.0,  # -0.5*abs(u)*u
            -0.3,  # -10*abs(p)*r
            0.08,  # -2*abs(q)*q
            -2.4,  # -4*abs(u)*r
        ],
        dtype=float,
    )
    np.testing.assert_allclose(wrench, expected, rtol=0.0, atol=1.0e-12)
    print("fossen_residual_wrench=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
