# Web dashboard and VLA recorder (2026-09-14)

The dashboard uses a dark flat layout with cyan/mint accents and translucent CSS panels. There are no new frontend packages, remote fonts, WebGL scenes, or animated backgrounds. Small, bounded blur is used only on the header and open dialog surfaces; reduced-transparency/motion preferences disable it. Isometric transforms and extrusion shadows were removed. Camera and sensor telemetry are central; telemetry instruments and pilot controls flank them. VLA collection has its own in-app dialog, opened from the header or collection launcher. Secondary tools and event/RC details collapse. Unchanged text and event rows are not rebuilt; closed RC/event panels skip rendering. RC polling and command timing are unchanged.

Mission FSM launch/monitor panels and their frontend handlers were removed. The HTTP mission-start command is rejected. Standalone mission implementations remain available outside this GUI. Existing pinger controls remain hidden.

## Collect a demonstration

1. Start the simulation, select the intended camera profile and flight mode, and wait for sensors and initial alignment.
2. Enter one precise task instruction and the matching collection mode in **VLA 시연 녹화**. Click **레코더 준비**. This launches the existing ROS collector with simulation time, task-demonstration checks, and the GUI RC span. It does not arm or issue RC commands.
3. ARM and enable pilot input yourself. Resolve the reported missing inputs. The start button requires a fresh status from the collector owned by this GUI session.
4. Click **녹화 시작**, demonstrate the task, then **성공 저장**, **실패 저장**, or **현재 녹화 폐기**. Reset/reposition only after finishing the episode. Success is operator-labelled, not automatically inferred.
5. Repeat within the same task/session. To change the instruction or collection mode, finish recording, click **세션 종료**, then prepare a new session. Closing the VLA window with its close button or Esc does not end the session or recording; the header shows the recording state.

Files are stored in `outputs/vla-demonstrations/sim_<UTC>_<id>/`: `collector.yaml`, `context.json`, tracked-source `source.patch`, `recorder.log`, and collector episodes under `staging/`. The context includes the commit, dirty status, selected/active scene information and available configuration content. The patch does not capture untracked source file contents; archive the working tree separately for exact reproducibility. Configuration snapshots are not measured robot calibration or live FCU parameter readback.

The GUI status service checks images, IMU, depth, RC, clock and task-demonstration preconditions. It reports optional DVL warnings separately. Source interruptions and discarded/saved episodes are visible through the collector result. A nominal camera rate does not establish actual frame freshness; inspect saved timestamps and validity before training. The GUI does not yet automate export or model training.

## Validation

The collector lifecycle suite passed 10 tests, including actual ROS service calls from the GUI adapter through start, sample capture and success save. These are synthetic contract fixtures, not robot task demonstrations. Controller tests use a recorder mock; the dedicated ROS test covers that boundary.

At this revision the combined run passed 23 tests and found two older expectations requiring follow-up: legacy pinger no-odometry defaults, and a mission-launch fixture that expects a process despite its current prerequisites. Neither changes the collector integration result. Browser verification at 1280 px found no horizontal overflow. Backend restart and `/api/status` verified recorder status availability. Real joystick/camera streaming performance and full task demonstrations were not exercised during this GUI refactor.


## Industrial UI refinement and checks

The 2026-09-14 refinement uses 2–3 px corners, thin borders, compact typography and ice-blue accents. Numeric inputs, virtual sticks, map backgrounds, instruments and auxiliary windows share the palette. A common window manager provides Esc, focus trapping, background inertness and focus return; nested raw editors preserve their parent window. Camera expansion moves the existing camera and pilot nodes into one dialog and restores their positions on close. This does not add RC publishers or change command timing.

Sensor values that are null/empty are displayed as missing, never 0. The new regression fails before the conversion fix and passes after it. Instruments show NO DATA without IMU/depth input; age exceeding 2 seconds is labelled STALE as a display hint, not a collector validity rule. Camera ages are backend receive ages, not an effective frame-rate measurement. Velocity is labelled with its reported source rather than being assumed to be a DVL measurement.

Browser checks covered: main screen; VLA open/close and actual collector prepare/idle-session close; disabled start with missing sensors; physics numeric inputs; map editor; camera expansion and restoration; Esc focus return. Test session `sim_20260913T170621Z_4eea932f` started no episode and saved no training frames. The simulation was stopped. The 23 relevant collector/controller tests passed; the two previously recorded legacy homing/mission tests were excluded from that targeted rerun. This UI work did not validate live robot operation or full camera-stream performance.

Run the display regression with Node.js:

```sh
node uuv_mujoco/current/tools/test_web_sensor_display.cjs
```

Main implementation: `gui/web_static/studio.css`, `studio.js`, `windows.js`, `sensors.js`, `recorder.js`, and the shared `app.js`. Existing dirty physics and startup changes were preserved.
