"""Read-only isolated FCU parameter verification via spare SITL UART."""

import json
import os
import time
from pathlib import Path

from pymavlink import mavutil

c = mavutil.mavlink_connection("udpin:0.0.0.0:14700", source_system=252)
assert c.wait_heartbeat(timeout=15) is not None
out = {}
for k in (
    "ATC_RAT_PIT_D",
    "ATC_RAT_RLL_D",
    "PSC_ACCZ_P",
    "MOT_PWM_MIN",
    "MOT_PWM_MAX",
    "FRAME_CONFIG",
    "RC3_TRIM",
    "JS_GAIN_DEFAULT",
    "SCHED_LOOP_RATE",
    "RC_SPEED",
    "SERVO_RATE",
):
    c.mav.param_request_read_send(1, 1, k.encode(), -1)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if m is not None and m.param_id == k:
            out[k] = float(m.param_value)
            break
    else:
        out[k] = None
p = (
    Path(
        os.environ.get(
            "REPLAY_OUTPUT_ROOT", "/workspace/outputs/research-pool-real2sim-20260916"
        )
    )
    / os.environ.get("CASE", "drag_fit")
    / "fcu_param_readback.json"
)
p.write_text(json.dumps(out, indent=2))
print(out)
c.close()
