# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compile the exact JSON clock expression and check the 2,500 us contract."""

from pathlib import Path
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "outputs/yaw-clock-regression"
OUT.mkdir(parents=True, exist_ok=True)
manifest = json.loads(
    (ROOT / "docs/assets/yaw-timing-20260915/clock_patch_manifest.json").read_text()
)
# Read the recorded baseline commit so this still reproduces after installing the fix.
original = subprocess.check_output(
    [
        "git",
        "-C",
        str(ROOT / "ardupilot_sub_stable"),
        "show",
        manifest["base_firmware_commit"] + ":libraries/SITL/SIM_JSON.cpp",
    ],
    text=True,
)
patched = original.replace(
    "time_now_us += deltat * 1.0e6;",
    "time_now_us += uint64_t(llround(deltat * 1.0e6));",
)
results = {}
for label, source in [("original", original), ("rounded", patched)]:
    expression = re.search(r"time_now_us \+= [^;]+;", source).group()
    cpp = r"""
#include <cmath>
#include <cstdint>
#include <cstdio>
int main() {
 double sim_t=0, last_pub=-1, last_timestamp_s=0;
 uint64_t time_now_us=0, short_ticks=0, ticks=0;
 for (int i=0;i<88000;i++) {
  sim_t+=0.00125;
  if (last_pub>=0 && sim_t+1e-9<last_pub+0.0025) continue;
  last_pub=sim_t;
  const double deltat=sim_t-last_timestamp_s;
  const uint64_t before=time_now_us;
  EXPRESSION
  last_timestamp_s=sim_t;
  if (ticks>0 && time_now_us-before!=2500) short_ticks++;
  ticks++;
 }
 printf("{\"ticks\":%llu,\"non_2500_us_ticks\":%llu,\"final_clock_us\":%llu}\n",(unsigned long long)ticks,(unsigned long long)short_ticks,(unsigned long long)time_now_us);
 return short_ticks?42:0;
}
""".replace("EXPRESSION", expression)
    p = OUT / f"clock_{label}.cpp"
    p.write_text(cpp)
    binary = OUT / f"clock_{label}"
    subprocess.run(["g++", "-std=c++11", "-O2", str(p), "-o", str(binary)], check=True)
    r = subprocess.run([str(binary)], capture_output=True, text=True)
    results[label] = {
        "expression": expression,
        "returncode": r.returncode,
        **json.loads(r.stdout),
    }
assert results["original"]["returncode"] == 42, "baseline must fail the clock contract"
assert results["rounded"]["returncode"] == 0, (
    "candidate must pass the same clock contract"
)
(OUT / "clock_regression.json").write_text(json.dumps(results, indent=2) + "\n")
print(json.dumps(results, indent=2))
