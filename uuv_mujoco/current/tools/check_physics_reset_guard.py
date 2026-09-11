"""A failed integration must stop before the controller sees a reset vehicle."""

from pathlib import Path
import sys

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.runtime.physics_step_guard import step_with_reset_guard

m = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>'
)
d = mujoco.MjData(m)
step_with_reset_guard(mujoco, m, d)
assert d.time > 0
# Trigger MuJoCo's actual numerical-warning / auto-reset path.
d.time = 1
d.qvel[0] = np.nan
stopped = False
try:
    if "--without_fix" in sys.argv:
        mujoco.mj_step(m, d)
    else:
        step_with_reset_guard(mujoco, m, d)
except RuntimeError as exc:
    stopped = True
    assert "Previous state:" in str(exc)
assert stopped, "physics reset silently continued into the next controller tick"
print("PASS numerical reset stops runtime and preserves a diagnostic state")
