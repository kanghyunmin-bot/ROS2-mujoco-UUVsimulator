# Development OS Compatibility Contract

Date: 2026-06-06

This contract keeps the same MuJoCo/SITL runtime path usable on the current
macOS workstation and on a later Ubuntu workstation.  It is a launch/runtime
contract, not a physics tuning document.

## Supported Runtime Shape

The active runtime has two supported host layouts:

| Host | SITL path | MuJoCo path | Viewer path |
| --- | --- | --- | --- |
| macOS development | Docker ArduSub SITL preferred | local Python env | `mjpython` when viewer is enabled |
| Ubuntu development | Docker or native ArduSub SITL | local Python env | Python with X11/Wayland display |

The launcher must select one Python/MuJoCo runtime consistently.  Do not let
GUI startup, headless probes, and plant replay use different Python
environments.

ROS2 Python probes launched from GUI helpers must use the selected environment's
`python`, not an unqualified `python3`.  On the current macOS workstation,
`conda run -n ros2_h311 ... python3` can resolve to `/usr/bin/python3` before
the conda interpreter, which makes `rclpy` look for the wrong C-extension ABI.
The GUI ROS shell contract is verified by `check_gui_ros_python_contract.py`.

## Required Environment Knobs

| Variable | Purpose |
| --- | --- |
| `MJ311_ROOT` | Preferred MuJoCo Python environment root; legacy name, not a required MuJoCo version |
| `MJ311_PYTHON` | Explicit Python interpreter override; actual MuJoCo version is checked at launch |
| `MJ311_MJPYTHON` | Explicit macOS viewer launcher override; actual MuJoCo version is checked at launch |
| `ROS_ENV_SETUP` | Explicit ROS2 setup file |
| `ROS_WORKSPACE_SETUP` | ROS workspace setup file for the active runtime |
| `UUV_RUNTIME_PROFILE` | Runtime rate profile: `low`, `balanced`, or `high` |
| `UUV_SITL_BACKEND` | `docker` or native SITL backend for GUI launches |
| `UUV_SKIP_DEV_OS_COMPAT_CHECK` | Set to `1` only to bypass launcher preflight during intentional debugging |
| `UUV_DEV_OS_COMPAT_STRICT` | Set to `1` to make warnings fail during migration/CI checks |

The default controller-contract runtime uses:

```text
SITL_SCHED_LOOP_RATE=400
SITL_SENSOR_HZ_DEFAULT=400
SITL_THRUSTER_LOOP_HZ_DEFAULT=400
SITL_DEDICATED_COMMAND_MAVLINK=1
```

Lower-rate profiles may be useful for laptop smoke tests, but they are not
controller-parity evidence unless the chosen rate is recorded with the result.

## Preflight Gates

Headless CI or plant replay:

```bash
python3 tools/check_dev_os_compat.py --headless
```

Interactive MuJoCo viewer:

```bash
python3 tools/check_dev_os_compat.py --require-viewer
```

Strict migration check:

```bash
python3 tools/check_dev_os_compat.py --headless --strict
```

Ubuntu migration check from any host:

```bash
python3 tools/check_dev_os_compat.py --headless --target-os ubuntu --strict
```

Interactive Ubuntu viewer readiness on the Ubuntu host:

```bash
python3 tools/check_dev_os_compat.py --require-viewer --target-os ubuntu
```

The gate checks:

- host OS and architecture,
- declared target OS contract,
- selected runtime Python,
- GUI ROS shell Python can import `rclpy`,
- MuJoCo import,
- macOS `mjpython` launcher when viewer is enabled,
- display availability for Ubuntu viewer sessions,
- Docker CLI, compose, and daemon reachability,
- Docker `host.docker.internal:host-gateway` mapping for Ubuntu,
- Ubuntu migration markers in the launcher, Docker SITL wrapper, and docs,
- launch script executability,
- active runtime alias availability: `uuv_mujoco/current -> v2.2`,
- runtime entrypoint availability: `run_uuv_mujoco.py` and legacy
  `run_urdf_full.py` wrapper,
- real/SITL parameter source availability,
- ROS2 shell sourcing state.

`launch_uuv_sim.sh` runs the same gate automatically with the Python/mjpython
launcher it selected for the run.  That keeps GUI Start, shell starts, macOS
viewer runs, and Ubuntu headless runs on the same compatibility contract.

## macOS Notes

macOS viewer launches must go through `mjpython`; headless runs use the selected
Python interpreter directly.  If GUI Start fails before the model opens, run:

```bash
python3 tools/check_dev_os_compat.py --require-viewer
```

If that reports a missing `mjpython_launcher`, set `MJ311_MJPYTHON` or install
MuJoCo into the environment referenced by `MJ311_ROOT`.

In restricted shells, `mjpython -c ...` can exit before printing even when the
runtime Python can import `mujoco.viewer`.  In that case the preflight reports a
warning instead of blocking launch.  A missing executable is still a failure.

The `MJ311_*` names are retained for backward compatibility with existing
scripts.  They do not mean the runtime must use MuJoCo 3.1.1.  Treat the
preflight output, for example `mujoco 3.6.0`, as the source of truth.

## Ubuntu Notes

Ubuntu viewer launches need a usable `DISPLAY` or `WAYLAND_DISPLAY`.  Headless
replay should pass without either.  Docker SITL must keep the compose
`extra_hosts` mapping:

```text
host.docker.internal:host-gateway
```

Without that mapping, Docker SITL cannot reliably reach the host MuJoCo JSON
sensor/servo ports on Linux.

For Ubuntu migration, the expected development flow is:

```bash
cd /path/to/uuv_sim/uuv_mujoco/current
python3 tools/check_dev_os_compat.py --headless --target-os ubuntu --strict
./launch_uuv_sim.sh --headless --sitl --ros2-real-pkg-compat
./launch_uuv_sim.sh --sitl --ros2-real-pkg-compat
```

The first command is the host contract gate.  The second isolates SITL/ROS2
without a viewer.  The third adds the MuJoCo viewer/display path only after the
headless contract is clean.

New launchers should call `run_uuv_mujoco.py`.  `run_urdf_full.py` remains as a
compatibility wrapper for saved command lines and older debug scripts.

## Non-Negotiable Parity Boundary

OS migration must not change the controller-parity observation surface:

```text
real /mavros/rc/out
vs
SITL MAVLink SERVO_OUTPUT_RAW telemetry
```

Likewise, closed-loop plant input remains backend-specific:

```text
Docker/MAVProxy: SITL JSON servo backend -> MuJoCo thruster input
Native/direct: SITL MAVLink SERVO_OUTPUT_RAW -> MuJoCo thruster input
```

If a host-specific workaround changes these surfaces, the run is invalid.
