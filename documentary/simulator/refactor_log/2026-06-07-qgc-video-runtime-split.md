# QGC Video Runtime Split

Date: 2026-06-07

Scope: `sim/current/sim/runtime`

## Change

`QgcVideoRuntime` now remains the public runtime wrapper while setup,
frame rendering, and cleanup are owned by focused helpers:

- `qgc_video_setup.py`: `qgc_video` enable checks, `stereo_left` availability,
  ffmpeg availability, ROS image bridge renderer sharing, renderer creation,
  and `QgcVideoStreamer` construction.
- `qgc_video_frame.py`: frame publish-rate gate, ROS shared renderer render,
  fallback to dedicated MuJoCo renderer, and RGB frame creation.
- `qgc_video_lifecycle.py`: streamer and renderer close behavior.
- `qgc_video.py`: public runtime API, per-frame schedule state, and failure
  disable policy.

## Contract

This is behavior-neutral.  It preserves:

- disabled behavior when `qgc_video` is false
- disabled behavior when `stereo_left` is missing
- disabled behavior when ffmpeg is unavailable
- renderer sharing via `ros_bridge.can_share_camera_renderer(...)`
- fallback to dedicated MuJoCo renderer if shared rendering fails
- frame-rate gate using `data.time + 1e-9 >= next_t`
- stream failure behavior: log, close streamer, and disable video

It does not change ArduPilot, SITL telemetry, RC override, plant input, ROS
topics, or physics coefficients.

## Verification

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current python3 <fake QGC video runtime smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_qgc_video_runtime_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_qgc_video_runtime_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Result: fake QGC runtime smoke `PASS`, source audit `fail=0 pass=11 warn=5`,
readiness `PASS`, thruster contract `OK`, physics contract audit completed,
and `sim/runtime/qgc_video.py` is no longer in the top 45 refactor inventory
hotspot list.
