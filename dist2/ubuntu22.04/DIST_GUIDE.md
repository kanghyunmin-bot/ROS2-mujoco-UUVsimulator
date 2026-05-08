# Dist2 Ubuntu 22.04 Distribution Guide

This guide is the mandatory release contract for every Ubuntu 22.04 dist2
package. It records the installation behavior that is known to work for the
`uuv_sim_ubuntu22.04_dist2` runtime package, plus the failure modes found while
repairing the distribution. Treat this file as a rulebook, not as optional
notes.

Any package that violates this guide is not a valid dist2 release. If the code
and this guide disagree, either fix the code to match the guide or update this
guide only after the new behavior has passed the same Ubuntu 22.04 install and
smoke tests documented below.

## Release Contract

Before creating or uploading a dist2 package, all of these rules must hold:

1. The default install path must remain native Ubuntu Python, not a virtualenv.
2. The exact documented install command must work on a clean or representative
   Ubuntu 22.04 machine.
3. `package_dist2.sh` must create the artifact from a clean release workflow or
   from an explicitly allowed dirty worktree test.
4. `verify_package.sh` must pass against the exact zip that will be shared.
5. Installer changes must be tested in Ubuntu 22.04 again, preferably with the
   Docker procedure in this file and, for GUI/QGroundControl, on real x86_64
   Ubuntu hardware.
6. Runtime scripts must not contain developer-machine absolute paths.
7. The package must not contain backup files, caches, logs, generated build
   outputs, `.git` directories, or macOS metadata.
8. The package must not block user-site Python packages with
   `PYTHONNOUSERSITE=1`.
9. The generated `.uuv_mujoco_env.sh` must not export non-existent executable
   paths such as an invalid `MJ311_MJPYTHON`.
10. Any missing external payload, such as real rosbag data, must be documented
    as external data and must not be silently assumed to exist inside the zip.

When changing the distribution, update this guide in the same change if the
change creates a new release rule, dependency rule, path rule, test rule, or
known caveat.

## Mandatory Release Checklist

Use this as the release gate. A single failed item blocks the package.

| ID | Area | Required check | Pass condition |
| --- | --- | --- | --- |
| R01 | Package source | Build with `package_from_github.sh` for release, or `package_dist2.sh --allow-dirty` only for local validation. | The exact zip to share exists under `dist2/ubuntu22.04/out/` and has a matching `SHA256SUMS`. |
| R02 | Installer default | Inspect `install_uuv_sim_ubuntu22.sh`. | `PYTHON_MODE="${PYTHON_MODE:-native}"` is still the default. |
| R03 | Python env | Source `.uuv_mujoco_env.sh` after install. | `UUV_PYTHON_MODE=native`, `MJ311_PYTHON=/usr/bin/python3`, no invalid `MJ311_MJPYTHON`. |
| R04 | User site | Search packaged files. | No `PYTHONNOUSERSITE=1` in release payload, except this guide and the audit document discussing the forbidden string. |
| R05 | ROS apt | Test on a machine/container that can already have ROS source files. | Installer reuses `ros2.sources` and does not create duplicate ROS apt entries. |
| R06 | Optional apt/ROS packages | Review apt install logic. | Optional packages use availability checks and missing optional packages only warn. |
| R07 | ArduPilot | Run installer as non-root. | ArduPilot prereqs run without the root-user refusal. |
| R08 | Setuptools | Inspect install order. | Installer pins `setuptools<80` again after ArduPilot prereqs and before `colcon build`. |
| R09 | Shell arrays | Search launch scripts. | No optional array expansion uses `"${ARRAY[@]-}"`; optional args are appended only when array length is non-zero. |
| R10 | Host paths | Search top-level payload and nested zips. | No `/Users/...`, `/home/kanghyunmin/...`, `file:///Users/...`, or build-machine absolute asset path. |
| R11 | External payloads | Check replay/data workflows. | Missing real rosbag data or large external inputs are documented and accepted through explicit arguments. |
| R12 | Nested zip hygiene | Inspect `uuv_mujoco.zip` and `rospkg/kmu26_auv.zip`. | No `.git`, cache, backup, logs, generated colcon output, or macOS metadata. |
| R13 | Static verification | Run `verify_package.sh` against the final zip. | All `[PASS]` lines complete and the verifier exits `0`. |
| R14 | Ubuntu install | Run the documented Ubuntu 22.04 install sequence. | Installer exits `0`, imports pass, colcon package builds. |
| R15 | Headless smoke | Run `timeout 90s ./launch_uuv_sim.sh --headless --no-ros2`. | Exit `0` or `124`; logs show the simulator reached runtime. |
| R16 | GUI/QGC caveat | Validate on real x86_64 Ubuntu when GUI/QGC is part of the release claim. | GUI starts with sourced env; QGroundControl is not judged from ARM64 Docker alone. |

Do not ship a package with "known small violations" of this checklist. If a
rule is obsolete, first change the code, validate the new behavior on Ubuntu
22.04, then update the rule with evidence.

## File-By-File Audit Requirements

Before packaging, audit these files and directories directly.

| Path | Required rule | Concrete failure to prevent |
| --- | --- | --- |
| `dist2/ubuntu22.04/install_uuv_sim_ubuntu22.sh` | Native Python default, ROS duplicate-source cleanup, optional package checks, non-root ArduPilot flow, post-ArduPilot `setuptools<80`, valid env file generation. | Installer works on the developer machine but fails on a fresh Ubuntu host. |
| `rospkg/dvl_msgs` | Must be packaged as `dvl_msgs.zip`, extracted by the installer, and built before `hit25_auv_ros2`. Do not rely on `ros-humble-dvl-msgs`; it is not available in the standard Humble apt repo used during validation. | `hit25_auv_ros2` fails at `find_package(dvl_msgs REQUIRED)`. |
| `rospkg/ping360_sonar_msgs` | Must be packaged as `ping360_sonar_msgs.zip`, extracted by the installer, and built before/with `hit25_auv_ros2`. | `/ping360/scan_echo` and `/ping360/echo` silently disappear because `SonarEcho` is unavailable. |
| `dist2/ubuntu22.04/package_dist2.sh` | Include this guide, `PORTABILITY_AUDIT.md`, manifest, installer, GUI entrypoints, nested runtime zips; exclude generated and local files recursively. | Zip contains stale backup files or misses required release docs. |
| `dist2/ubuntu22.04/verify_package.sh` | Check required top-level files, nested required runtime files, syntax, forbidden paths, backup/cache files, and user-site blocking env. | A bad package passes local packaging because only top-level files were checked. |
| `run_control_gui.sh` | Source the generated env when present, use native user site packages, do not set `PYTHONNOUSERSITE=1`, resolve paths from the script location. | GUI starts on the developer machine but cannot import installed Python packages on Ubuntu. |
| `uuv_control_gui.py` | Locate bundled runtime paths relative to the extracted package or explicit environment variables. | GUI points to an old local checkout instead of the packaged runtime. |
| `uuv_mujoco/v2.2/launch_uuv_sim.sh` | Build command arrays explicitly, support `--headless --no-ros2`, no blank optional args. | Python exits with `run_urdf_full.py: error: unrecognized arguments:`. |
| `uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh` | Source ROS/env safely, wait for runtime readiness, pass optional args without blank strings. | SITL launch fails before simulator readiness. |
| `uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh` | Resolve `ARDUPILOT_DIR`, construct SITL command with array-length checks, do not assume local ArduPilot checkout. | SITL command uses missing local paths or receives malformed arguments. |
| `uuv_mujoco/v2.2/reset_uuv_sim.sh` | Use runtime-relative paths and safe shell syntax. | Reset works only from one current working directory. |
| `uuv_mujoco/v2.2/assets/**` | Mesh and JSON/XML references must be relative to bundled assets. | MuJoCo fails to load assets on another computer. |
| `rospkg/kmu26_auv/urdf/**` | URDF mesh paths must be package-relative or relative; no `file:///Users/...`. | RViz/ROS cannot load robot description outside the developer Mac. |
| `rospkg/kmu26_auv/rviz/**` | RViz config must use topics or empty/default description fields, not absolute local URDF files. | RViz opens with a broken robot model path. |
| `document/docsource/*.sh` | Compute `SCRIPT_DIR` and repository/package root from `BASH_SOURCE[0]`; accept external bag paths explicitly. | Documentation helper scripts run only from the original checkout path. |
| `document/docsource/*.py` | Use `Path(__file__).resolve()` or explicit CLI/env paths for local resources. | Analysis/replay scripts read files from a developer-only absolute path. |

If a future file becomes part of the release entrypoint surface, add it to this
table before shipping.

## Exact Path Rewrite Rules

Use these concrete rewrite rules whenever host-specific paths are found:

```text
BAD:  /Users/kanghyunmin/Desktop/uuv_sim/...
GOOD: path derived from the current script, extracted package root, or env var

BAD:  file:///Users/kanghyunmin/.../mesh.stl
GOOD: package://kmu26_auv/... or a relative mesh path that exists in the zip

BAD:  /tmp/uuvdist... inside a packaged script or config
GOOD: no packaging temp path in release payload

BAD:  hard-coded rosbag path in replay script
GOOD: required --bag argument or documented environment variable

BAD:  hard-coded ArduPilot checkout under the development workspace
GOOD: ${ARDUPILOT_DIR:-$HOME/ardupilot}
```

Preferred shell root pattern:

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
```

Preferred Python root pattern:

```python
from pathlib import Path

HERE = Path(__file__).resolve().parent
```

Do not replace a hard-coded path with another hard-coded path for the current
machine. The release must survive a different username, a different extraction
directory, and a clean Ubuntu install.

## Required Search Commands

Run these from the workspace root before packaging:

```bash
rg -n --hidden \
  --glob '!dist2/ubuntu22.04/out/**' \
  --glob '!**/.git/**' \
  --glob '!**/__pycache__/**' \
  '/Users/|/home/kanghyunmin|file:///Users|/tmp/uuvdist|PYTHONNOUSERSITE=1' \
  dist2 run_control_gui.sh uuv_control_gui.py uuv_mujoco rospkg document/docsource
```

Allowed hits are only documentation that explains forbidden strings, such as
this guide and `PORTABILITY_AUDIT.md`. Any runtime script, config, JSON, XML,
URDF, RViz, or Python file hit must be fixed before packaging.

Search for unsafe optional shell array expansion:

```bash
rg -n '\$\{[A-Za-z0-9_]+\[@\]-\}' \
  dist2/ubuntu22.04/*.sh \
  uuv_mujoco/v2.2/*.sh \
  document/docsource/*.sh
```

Expected result: no matches.

After packaging, audit the exact nested zips when debugging a portability
failure:

```bash
rm -rf /tmp/uuvdist2_audit
mkdir -p /tmp/uuvdist2_audit/root /tmp/uuvdist2_audit/runtime /tmp/uuvdist2_audit/rospkg
unzip -q dist2/ubuntu22.04/out/uuv_sim_ubuntu22.04_dist2.zip -d /tmp/uuvdist2_audit/root
unzip -q /tmp/uuvdist2_audit/root/uuv_sim_ubuntu22.04_dist2/uuv_mujoco.zip -d /tmp/uuvdist2_audit/runtime
unzip -q /tmp/uuvdist2_audit/root/uuv_sim_ubuntu22.04_dist2/rospkg/kmu26_auv.zip -d /tmp/uuvdist2_audit/rospkg
rg -n '/Users/|/home/kanghyunmin|file:///Users|PYTHONNOUSERSITE=1' /tmp/uuvdist2_audit
```

Expected result: hits only in release guide/audit documentation, not runtime
payload.

## Known-Good Reference

The working reference package tested on Ubuntu was:

```text
uuv_sim_ubuntu22.04_dist2.zip from the known-good Ubuntu test package
sha256: 6f1b9a329e44a6c8d2c9e8c50c6df702d5eabf3d86a4fe3857202125f47fb783
```

The install command sequence that succeeded was:

```bash
sudo apt-get update
sudo apt-get install -y unzip
unzip uuv_sim_ubuntu22.04_dist2.zip
cd uuv_sim_ubuntu22.04_dist2
chmod +x install_uuv_sim_ubuntu22.sh
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
source ./.uuv_mujoco_env.sh
```

The GUI in that package still had some incomplete behavior, but the Ubuntu
runtime installation itself progressed correctly. When installation starts
failing again, compare the current dist2 installer against the behavior in this
guide before debugging unrelated runtime code.

## Current Validated Candidate

The repaired local candidate built after the 2026-05-08 Docker validation is:

```text
dist2/ubuntu22.04/out/uuv_sim_ubuntu22.04_dist2.zip
dist2/ubuntu22.04/out/SHA256SUMS
```

Use `SHA256SUMS` next to the generated zip for the current artifact hash. Do
not hard-code the current zip hash inside this guide because this guide is part
of the package payload; editing the embedded hash changes the zip hash again.

Validation performed:

- `package_dist2.sh` completed.
- `verify_package.sh` passed.
- Ubuntu 22.04 Docker install completed as a non-root user.
- Native Python runtime imports passed after sourcing `.uuv_mujoco_env.sh`.
- Direct headless MuJoCo smoke test reached the running simulation state and
  exited through `timeout` status `124`, which is acceptable for a long-running
  simulator.

Docker validation logs from this repair pass were written to:

```text
/tmp/uuv_docker_ubuntu2204_test_user/full_test.log
/tmp/uuv_docker_ubuntu2204_test_user/launch_headless.log
/tmp/uuv_docker_ubuntu2204_test_user_patched/launch_headless_patched.log
```

Those `/tmp` logs are local evidence, not release payload. Do not package them.

## Non-Negotiable Installer Rules

The default Python install path must stay native Python first:

```bash
PYTHON_MODE="${PYTHON_MODE:-native}"
```

Native mode means:

- Use the system `python3`, normally `/usr/bin/python3` on Ubuntu 22.04.
- Install runtime Python packages into the user site with `pip --user`.
- Do not require a MuJoCo virtualenv for the default install path.
- Write `.uuv_mujoco_env.sh` so `MJ311_PYTHON` points to native `python3`.
- In native mode, unset `MJ311_ROOT` in the generated env file.

Virtualenv mode is still allowed, but only as an explicit opt-in:

```bash
./install_uuv_sim_ubuntu22.sh --python-mode venv --venv-root ~/.venvs/uuv_mujoco --noninteractive
```

Do not make venv the default unless the Ubuntu install path is revalidated from
a clean machine. The previous failure happened after the dist2 installer drifted
from the known-good native Python behavior toward venv-first behavior.

Keep the MuJoCo pip dependency pinned by default:

```bash
MUJOCO_PIP_SPEC="${MUJOCO_PIP_SPEC:-mujoco==3.8.0}"
```

The installer should keep these options:

```text
--native-python
--python-mode native|venv
--venv-root PATH
--mujoco-pip-spec SPEC
--noninteractive
--mask-modemmanager
--run-after-install
--run-headless
--run-gui
```

The run-after-install options are useful, but they must not force venv mode.

## ROS Apt Repository Handling

Ubuntu machines may already have ROS 2 configured through:

```text
/etc/apt/sources.list.d/ros2.sources
```

The installer must detect that case and avoid adding a duplicate
`ros2.list` entry. Duplicate ROS apt sources can cause apt update/install
problems.

Required behavior:

- If `ros2.sources` exists and contains `packages.ros.org/ros2/ubuntu`, reuse it.
- If both `ros2.sources` and `ros2.list` exist for the ROS repository, remove
  the duplicate `ros2.list`.
- Only create `/etc/apt/sources.list.d/ros2.list` when no existing ROS source is
  present.

The relevant functions are:

```bash
ensure_ros_apt_repo
cleanup_duplicate_ros_apt_repo
```

Do not remove these functions just because a clean VM works without them. They
matter on machines that have already installed ROS 2 through a different setup
path.

## Apt Dependency Policy

Base packages can be installed with a normal `apt-get install -y` block. Optional
or environment-dependent packages must use an availability check.

Keep this pattern:

```bash
apt_install_available "label" pkg1 pkg2 pkg3
```

Use it for packages that can vary by Ubuntu point release, repository state, or
host configuration:

- OpenGL/headless helpers
- QGroundControl Qt/AppImage helpers
- GStreamer video helpers
- diagnostic tools
- SocketCAN tools

ROS packages must also be checked with `apt_package_exists` before installation.
Do not hard-fail the whole install just because one optional ROS package is
missing from apt. Install the packages that are available and log warnings for
the missing ones.

## Colcon And Setuptools Compatibility

Before building the bundled ROS helper package, keep the setuptools compatibility
step:

```bash
python3 -m pip install --user 'setuptools<80'
```

This exists because some Ubuntu/ROS/colcon combinations fail after newer
setuptools changes. Removing it can make `colcon build` fail even when ROS itself
is installed correctly.

When sourcing ROS setup inside the installer, keep `set +u` around the source:

```bash
(
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  cd "$ros_ws"
  colcon build --symlink-install --packages-select hit25_auv_ros2
)
```

ROS setup scripts are not guaranteed to be safe under `set -u`. If this is
removed, the installer can fail before the package build starts.

## Python Dependency Install

Native mode should install:

```bash
python3 -m pip install --user -U pip packaging "setuptools<80" wheel
python3 -m pip install --user -U \
  rosbags python-pptx mujoco==3.8.0 pymavlink MAVProxy pexpect pillow \
  future dronecan gnureadline "empy==3.3.4"
```

Venv mode should install the same runtime packages inside the virtualenv, plus
the packages already available through apt where needed:

```bash
"$MJ311_ROOT/bin/python" -m pip install -U pip "setuptools<80" wheel
"$MJ311_ROOT/bin/python" -m pip install \
  numpy matplotlib rosbags python-pptx mujoco==3.8.0 pymavlink MAVProxy \
  pexpect pillow future dronecan gnureadline "empy==3.3.4"
```

After installation, verify imports in the same interpreter that will be used by
the runtime:

```python
import mujoco
import numpy
import matplotlib
import rosbags
import pptx
import pymavlink
import MAVProxy
import pexpect
import PIL
import future
import dronecan
import em
```

## Generated Environment File

The installer writes:

```text
.uuv_mujoco_env.sh
```

In native mode, it must behave like this:

```bash
export UUV_PYTHON_MODE="native"
unset MJ311_ROOT
export MJ311_PYTHON="/usr/bin/python3"
```

It should also clear invalid `MJ311_MJPYTHON` values:

```bash
if [[ -n "${MJ311_MJPYTHON:-}" && ! -x "${MJ311_MJPYTHON}" ]]; then
  unset MJ311_MJPYTHON
fi
```

In venv mode, it should behave like this:

```bash
export UUV_PYTHON_MODE="venv"
export MJ311_ROOT="${MJ311_ROOT:-$HOME/.venvs/uuv_mujoco}"
export MJ311_PYTHON="${MJ311_PYTHON:-${MJ311_ROOT}/bin/python}"
if [[ -z "${MJ311_MJPYTHON:-}" && -x "${MJ311_ROOT}/bin/mjpython" ]]; then
  export MJ311_MJPYTHON="${MJ311_ROOT}/bin/mjpython"
fi
```

Do not export a non-existent `mjpython` path unconditionally. Some launch scripts
will treat a bad `MJ311_MJPYTHON` as an error.

## Runtime Path And Portability Rules

Every file inside a release zip must work after extraction on another Ubuntu
machine, regardless of the developer account name or checkout path.

Allowed path sources:

- Paths derived from the current script location, usually with
  `BASH_SOURCE[0]`.
- Paths derived from the extracted dist2 package root.
- Paths written by the installer into `.uuv_mujoco_env.sh`.
- User overrides documented by the installer, such as `ARDUPILOT_DIR`,
  `ROS_ENV_SETUP`, `ROS_INSTALL_SETUP`, `QGC_APP`, `MJ311_PYTHON`, and
  `MJ311_MJPYTHON`.

Forbidden path patterns in release payloads:

- `/Users/kanghyunmin`
- `/Users/`
- `/home/kanghyunmin`
- development-only absolute checkout paths
- temporary packaging paths such as `/tmp/uuvdist`
- absolute RViz, URDF, mesh, rosbag, or JSON asset paths that only exist on the
  build machine

Rules for runtime files:

- Shell entrypoints must compute their root relative to the script file instead
  of assuming the caller's current directory.
- Python modules must locate bundled assets relative to `__file__` or an
  explicit environment variable, not a developer checkout path.
- RViz config should prefer ROS package URLs, topics, or empty defaults over
  absolute host file paths.
- URDF and mesh references in packaged ROS content must use package-relative or
  relative paths. Do not use `file:///Users/...`.
- MuJoCo JSON and XML assets must reference bundled files by relative paths
  that remain valid after extraction.
- Replay scripts must accept an explicit bag path argument when real rosbag data
  is not bundled.

Specific portability fixes found during dist2 repair:

- `uuv_mujoco/v2.2/assets/ping360/ping360_body_no_cable.json` must reference
  the bundled `PING360_SONAR_BR-100399/PING360-ASM.STL` asset relatively.
- `rospkg/kmu26_auv/urdf/rov_rviz_local.urdf` must not contain a
  `file:///Users/...` mesh path.
- `rospkg/kmu26_auv/rviz/nuc_live.rviz` must not point to an absolute local
  URDF description file.
- `document/docsource` helper scripts must derive the repository root from the
  script path instead of hard-coding `/Users/kanghyunmin/...`.
- `document/docsource/run_closed_loop_april1_replay.sh` must document that real
  rosbag data is external and must be supplied with `--bag` when not included.

## ArduPilot Policy

The dist2 package does not include an ArduPilot checkout. The installer clones
upstream ArduPilot:

```bash
git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
git -C "$ARDUPILOT_DIR" submodule update --init --recursive
```

This installer does not patch ArduPilot. If a future simulation behavior depends
on a local ArduPilot patch, that patch must be documented and applied explicitly.
Do not assume local submodule edits from the development workspace are present in
the Ubuntu dist2 install.

ArduPilot's upstream prereq script must run as a normal user, not as root:

```text
Tools/environment_install/install-prereqs-ubuntu.sh
```

It exits immediately when UID is `0`. This matters in Docker because a container
starts as root by default. The installer itself may use `sudo` internally for apt
and system setup, but the top-level installer process must be launched from the
target user account.

When adding the user to groups such as `dialout`, do not blindly trust
`SUDO_USER`. In Docker, `sudo -Hiu uuv` can leave `SUDO_USER=root`, even though
the effective install user is `uuv`. The target user selection rule is:

- use `SUDO_USER` only when it is non-empty and not `root`
- otherwise use `id -un`

After ArduPilot's prereq script runs, re-pin Python build tooling before colcon:

```bash
python3 -m pip install --user 'setuptools<80'
```

During validation, the ArduPilot prereq step upgraded setuptools to a newer
version, then `colcon build` warned about setuptools compatibility. The installer
must restore the `<80` pin before building `hit25_auv_ros2`.

`dvl_msgs` is not optional for `hit25_auv_ros2`. The apt package
`ros-humble-dvl-msgs` was unavailable during Docker validation, so the dist must
bundle a source zip for `rospkg/dvl_msgs` and extract it into the same colcon
workspace before `kmu26_auv`. The build package set must include:

```bash
colcon build --symlink-install --packages-select \
  dvl_msgs ping360_sonar_msgs hit25_auv_ros2
```

It is acceptable for the installer to warn and continue when the apt package is
missing, but it is not acceptable for the final colcon workspace to lack
`dvl_msgs`.

## ModemManager

Keep `--mask-modemmanager` available:

```bash
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
```

This stops and masks `ModemManager.service` when present. It matters for serial
devices used by QGroundControl, MAVROS, and related hardware workflows.

## Packaging Workflow

Preferred release packaging should come from a clean GitHub checkout:

```bash
./dist2/ubuntu22.04/package_from_github.sh --branch uuv_sim
```

Expected output:

```text
dist2/ubuntu22.04/out/latest/
  uuv_sim_ubuntu22.04_dist2_uuv_sim_<commit>_<date>.zip
  SHA256SUMS
  RELEASE_MANIFEST.txt
  README_UPLOAD.txt
```

Use current worktree packaging only for deliberate local tests:

```bash
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty
./dist2/ubuntu22.04/verify_package.sh
```

The package script intentionally excludes:

- large rosbag data
- full document tree
- ArduPilot checkout
- QGroundControl binaries
- generated install artifacts
- `.git`, `__pycache__`, `.pyc`, logs, local backups, and macOS metadata

## Strict Release Build Procedure

For a local validation build from the current worktree:

```bash
cd /path/to/uuv_sim
bash -n dist2/ubuntu22.04/install_uuv_sim_ubuntu22.sh
bash -n dist2/ubuntu22.04/package_dist2.sh
bash -n dist2/ubuntu22.04/verify_package.sh
rg -n '\$\{[A-Za-z0-9_]+\[@\]-\}' dist2/ubuntu22.04/*.sh uuv_mujoco/v2.2/*.sh document/docsource/*.sh
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_test_out
./dist2/ubuntu22.04/verify_package.sh /tmp/uuvdist2_test_out/uuv_sim_ubuntu22.04_dist2.zip
```

The `rg` command above must print nothing. If it prints a file, fix that file
before packaging.

For a release candidate from GitHub:

```bash
cd /path/to/uuv_sim
./dist2/ubuntu22.04/package_from_github.sh --branch uuv_sim
```

Then verify the exact zip produced under:

```text
dist2/ubuntu22.04/out/latest/
```

Do not verify one zip and upload another. The path passed to
`verify_package.sh`, the path listed in `SHA256SUMS`, and the uploaded file must
be the same artifact.

Minimum release evidence to keep:

```text
package command used
zip path
SHA256SUMS contents
verify_package.sh output
Ubuntu/Docker install log
headless smoke log
GUI/QGC host-test note, if GUI/QGC behavior is claimed
```

Never edit files inside the staged `out/uuv_sim_ubuntu22.04_dist2/` directory by
hand. Fix the source file, rebuild the package, and verify again.

## Release Artifact Rules

The top-level release zip should contain only the files required to install and
run the dist2 package. Expected top-level payload:

```text
uuv_sim_ubuntu22.04_dist2/
  install_uuv_sim_ubuntu22.sh
  run_control_gui.sh
  package metadata and guides
  RELEASE_MANIFEST.txt
  DIST_GUIDE.md
  PORTABILITY_AUDIT.md
  uuv_mujoco.zip
  rospkg/kmu26_auv.zip
  rospkg/ping360_sonar_msgs.zip
  document/docsource/...
```

Required release documents:

- `DIST_GUIDE.md`: this mandatory rulebook.
- `PORTABILITY_AUDIT.md`: current portability audit and path policy.
- `RELEASE_MANIFEST.txt`: generated source metadata for the zip.
- `rospkg/ping360_sonar_msgs.zip`: custom Ping360 `SonarEcho` message package.

The release zip must not include:

- ArduPilot checkout
- QGroundControl AppImage or extracted QGroundControl files
- full rosbag data sets unless a release explicitly documents them
- `build`, `install`, `log`, or colcon generated outputs
- `.git`, `.github`, `.DS_Store`, `__MACOSX`, `__pycache__`, `.pytest_cache`
- `*.pyc`, `*.log`, `*.bak`, `*.bak_*`, `*~`, `*.orig`
- local Docker logs or temporary validation work directories

Nested zips must follow the same exclusion rules. A clean top-level zip is not
enough if `uuv_mujoco.zip` or `rospkg/kmu26_auv.zip` still contains backups,
caches, `.git`, or host paths.

When writing package output during local testing, prefer an absolute `--out-dir`:

```bash
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_test_out
```

Do not rely on a relative output path while the package script is changing
directories internally. Use the default release workflow or an absolute output
directory so zip creation and later verification read the same artifact.

## Verification Before Upload

From the workspace root:

```bash
bash -n dist2/ubuntu22.04/install_uuv_sim_ubuntu22.sh
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_test_out
./dist2/ubuntu22.04/verify_package.sh /tmp/uuvdist2_test_out/uuv_sim_ubuntu22.04_dist2.zip
```

Expected verification output includes:

```text
[PASS] top-level package paths exist
[PASS] top-level shell syntax ok
[PASS] installer includes current dependency and run-option markers
[PASS] release manifest records source metadata
[PASS] uuv_mujoco.zip excludes generated/cache/backup files
[PASS] kmu26_auv.zip excludes git/generated/cache/backup files
[PASS] dist2 archive excludes large external assets
[PASS] package excludes host-specific paths and user-site blocking env
[PASS] nested runtime paths exist
[PASS] nested shell syntax ok
[PASS] python syntax ok
```

This only proves package structure and syntax. It does not prove a real Ubuntu
install. A real Ubuntu test is still required after installer changes.

## Ubuntu Install Test

On a clean or representative Ubuntu 22.04 host:

```bash
sudo apt-get update
sudo apt-get install -y unzip
unzip uuv_sim_ubuntu22.04_dist2.zip
cd uuv_sim_ubuntu22.04_dist2
chmod +x install_uuv_sim_ubuntu22.sh
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
source ./.uuv_mujoco_env.sh
```

Then check:

```bash
echo "$UUV_PYTHON_MODE"
echo "$MJ311_PYTHON"
python3 - <<'PY'
import mujoco
import rosbags
import pymavlink
import MAVProxy
import em
print("runtime imports ok")
PY
```

Expected defaults:

```text
UUV_PYTHON_MODE=native
MJ311_PYTHON=/usr/bin/python3
```

Headless smoke test:

```bash
cd uuv_mujoco/v2.2
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

Direct MuJoCo-only smoke test:

```bash
cd uuv_mujoco/v2.2
timeout 90s ./launch_uuv_sim.sh --headless --no-ros2
```

Expected result for the direct smoke test is either:

- exit `124` from `timeout`, with the simulation still running until killed
- exit `0` if the runtime is changed to terminate by itself

It must not fail with:

```text
run_urdf_full.py: error: unrecognized arguments:
```

That specific blank-argument failure means a shell array was expanded with a
default such as `"${EXTRA_ARGS[@]-}"`, which can forward an empty string to
Python. Use an explicit array-length check before appending optional arguments.

GUI test:

```bash
cd ../..
./run_control_gui.sh
```

## Smoke Test Acceptance Criteria

An installer validation is acceptable only when these checks pass:

- The installer exits successfully.
- `.uuv_mujoco_env.sh` exists and can be sourced in a fresh shell.
- `UUV_PYTHON_MODE` is `native` by default.
- `MJ311_PYTHON` points to an executable system Python, normally
  `/usr/bin/python3`.
- Native Python can import `mujoco`, `rosbags`, `pymavlink`, `MAVProxy`,
  `pexpect`, `PIL`, `future`, `dronecan`, and `em`.
- ROS Humble setup is present under `/opt/ros/humble`.
- The `hit25_auv_ros2` colcon package builds successfully.
- The direct MuJoCo headless test reaches runtime logs such as headless mode,
  profile loading, thruster configuration, and physics setup.
- A `timeout` exit code of `124` is accepted for a long-running simulator.
- Immediate Python argparse failure is not accepted.

The following failure is specifically blocked:

```text
run_urdf_full.py: error: unrecognized arguments:
```

That message means an empty string was passed as a command-line argument. Shell
arrays must be appended with explicit length checks:

```bash
cmd=(python3 run_urdf_full.py)
if (( ${#EXTRA_ARGS[@]} > 0 )); then
  cmd+=("${EXTRA_ARGS[@]}")
fi
exec "${cmd[@]}"
```

Do not use this pattern for optional arrays:

```bash
exec python3 run_urdf_full.py "${EXTRA_ARGS[@]-}"
```

## Docker Ubuntu 22.04 Validation

The package was validated in Docker on 2026-05-08 using `ubuntu:22.04`.
On Apple Silicon Docker this is Ubuntu 22.04 ARM64, not x86_64. That still
validates the Ubuntu 22.04 apt/Python/ROS/MuJoCo install path, but an x86_64
release check should be run with `--platform linux/amd64` or on real x86_64
hardware when architecture-specific behavior matters.

Do not run the installer as `root` in Docker. ArduPilot's upstream
`Tools/environment_install/install-prereqs-ubuntu.sh` exits immediately when
UID is `0`:

```text
Please do not run this script as root; don't sudo it!
```

Use a normal user with passwordless sudo in the container. Minimal Docker
bootstrap:

```bash
docker run -i --name uuv-dist2-ubuntu2204-test \
  -e DEBIAN_FRONTEND=noninteractive \
  -v "$PWD/out/uuv_sim_ubuntu22.04_dist2.zip:/tmp/uuv_sim_ubuntu22.04_dist2.zip:ro" \
  ubuntu:22.04 bash -s <<'ROOT_SCRIPT'
set -euo pipefail
apt-get update
apt-get install -y unzip ca-certificates tzdata sudo
useradd -m -s /bin/bash uuv
echo "uuv ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/uuv
chmod 0440 /etc/sudoers.d/uuv
cp /tmp/uuv_sim_ubuntu22.04_dist2.zip /home/uuv/
chown uuv:uuv /home/uuv/uuv_sim_ubuntu22.04_dist2.zip
sudo -Hiu uuv bash -s <<'USER_SCRIPT'
set -euo pipefail
cd "$HOME"
unzip -q uuv_sim_ubuntu22.04_dist2.zip
cd uuv_sim_ubuntu22.04_dist2
chmod +x install_uuv_sim_ubuntu22.sh
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
source ./.uuv_mujoco_env.sh
python3 - <<'PY'
import mujoco, rosbags, pymavlink, MAVProxy, pexpect, PIL, future, dronecan, em
print("runtime imports ok")
PY
cd uuv_mujoco/v2.2
set +e
timeout 90s ./launch_uuv_sim.sh --headless --no-ros2
status=$?
set -e
if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
  exit "$status"
fi
USER_SCRIPT
ROOT_SCRIPT
```

Keep `-i` on `docker run` when feeding heredocs. Without stdin attached, the
inner scripts can be truncated or not executed. Prefer heredocs over nested
single-line quoted commands because the installer logs are long and quoting
mistakes hide the real failure.

Docker-specific service warnings such as `policy-rc.d denied execution` or
missing system dbus are expected during apt installs and are not installer
failures by themselves.

Docker validation caveats:

- Docker on Apple Silicon validates Ubuntu 22.04 ARM64 unless
  `--platform linux/amd64` is used.
- MuJoCo pip wheels are available on ARM64 and the native Python runtime can be
  validated there.
- The downloaded QGroundControl AppImage is x86_64, so GUI/QGC execution must be
  checked on x86_64 Ubuntu or an explicit amd64 Docker setup.
- Docker has no normal desktop session, system dbus, or GPU/USB access by
  default. Treat GUI, serial, and joystick behavior as separate host tests.

## Missed-Issue Log From The 2026-05-08 Repair

This section is an error ledger. These are the concrete issues that were missed
or underestimated during the dist2 repair and then showed up as install,
packaging, smoke-test, or portability problems. Future release work must check
each item explicitly instead of relying on memory.

### M01 - Reviewed The Wrong Surface First

What was missed:

- The first analysis paid too much attention to general runtime files and
  `rospkg` details.
- The user clarified that the important comparison was the dist2 release package
  and especially the installer behavior.

Why it mattered:

- `rospkg` portability is still important, but it was not the first reason the
  Ubuntu install failed.
- The known-good zip worked because its dist2 installer followed a different
  install path.

Permanent rule:

- When a known-good dist2 zip exists, compare these first:
  1. `install_uuv_sim_ubuntu22.sh`
  2. `.uuv_mujoco_env.sh` generation logic
  3. `package_dist2.sh`
  4. `verify_package.sh`
  5. top-level launcher scripts
- Only after the installer path is understood should runtime package details be
  debugged.

### M02 - Native Python Default Was The Critical Difference

What was missed:

- The broken installer drifted away from the known-good native Python default.
- The known-good package installed runtime Python dependencies into the Ubuntu
  user's site-packages with `pip --user`.

Observed failure class:

- Ubuntu install could complete some system steps but later runtime imports or
  launch behavior did not match the known-good zip.

Root cause:

- The default path became too close to a MuJoCo virtualenv-first strategy.
- That changed interpreter selection, package visibility, and generated env
  behavior.

Fix applied:

- Keep the default:

```bash
PYTHON_MODE="${PYTHON_MODE:-native}"
```

- In native mode, write:

```bash
export UUV_PYTHON_MODE="native"
unset MJ311_ROOT
export MJ311_PYTHON="/usr/bin/python3"
```

Permanent rule:

- Virtualenv mode may exist only as explicit opt-in.
- Any future change that makes venv default requires a fresh clean Ubuntu 22.04
  install test and this guide must be updated with evidence.

### M03 - `PYTHONNOUSERSITE=1` Broke The Native Strategy

What was missed:

- `run_control_gui.sh` contained environment behavior that could block user-site
  packages.

Observed failure class:

- Packages installed with `python3 -m pip install --user ...` are invisible to
  GUI or child Python processes.

Root cause:

- Native mode depends on user-site packages.
- `PYTHONNOUSERSITE=1` disables exactly the location where native mode installs
  the required packages.

Fix applied:

- Remove `PYTHONNOUSERSITE=1` from runtime GUI launch behavior.
- Add verifier scan so packaged runtime files cannot silently reintroduce it.

Permanent rule:

- If native mode installs with `pip --user`, release runtime launchers must not
  disable user-site packages.

### M04 - Invalid `MJ311_MJPYTHON` Was Treated Like A Valid Runtime

What was missed:

- Some launch behavior assumed `MJ311_MJPYTHON` was useful if the variable was
  set, even when the path did not exist.

Observed failure class:

- Launch scripts can fail before falling back to normal Python.
- Native Linux installs do not require `mjpython` by default.

Root cause:

- The env file exported or preserved a path that was not executable.

Fix applied:

- Generated env now clears invalid `MJ311_MJPYTHON`:

```bash
if [[ -n "${MJ311_MJPYTHON:-}" && ! -x "${MJ311_MJPYTHON}" ]]; then
  unset MJ311_MJPYTHON
fi
```

- In venv mode, `MJ311_MJPYTHON` is exported only if
  `${MJ311_ROOT}/bin/mjpython` exists and is executable.

Permanent rule:

- Never export an executable env var unless the target exists and has execute
  permission.

### M05 - Docker Installer Was First Run As Root

What was missed:

- Docker starts as root by default.
- ArduPilot's upstream prereq script refuses root execution.

Observed error:

```text
Please do not run this script as root; don't sudo it!
```

Root cause:

- The first Docker validation launched the installer from UID `0`.
- ArduPilot expects a normal user and uses sudo internally where needed.

Fix applied:

- Docker validation creates a normal `uuv` user.
- The user gets passwordless sudo.
- The installer runs under `sudo -Hiu uuv`, not directly as root.

Permanent rule:

- A root-run Docker install test is invalid for this package.
- Always validate ArduPilot prereqs from a non-root account.

### M06 - `SUDO_USER=root` In Docker Pointed At The Wrong User

What was missed:

- After switching users in Docker with `sudo -Hiu uuv`, `SUDO_USER` can still be
  `root`.

Observed failure class:

- Group setup such as `dialout` can target `root` instead of the actual install
  user.

Root cause:

- Installer logic trusted `SUDO_USER` without checking whether it was useful.

Fix applied:

- The target user selection rule became:

```bash
if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
  target_user="${SUDO_USER}"
else
  target_user="$(id -un)"
fi
```

Permanent rule:

- `SUDO_USER` is advisory, not authoritative.
- If `SUDO_USER` is empty or `root`, use the effective username from `id -un`.

### M07 - ArduPilot Prereqs Changed Python Build Tooling

What was missed:

- ArduPilot's prereq script can install or upgrade Python packages after the
  installer already prepared Python tooling.

Observed behavior:

- During Docker validation, setuptools was upgraded to a newer version.
- `colcon build` then warned about setuptools compatibility.

Root cause:

- The installer pinned `setuptools<80`, then a later external prereq step moved
  it again.

Fix applied:

- Re-run:

```bash
python3 -m pip install --user 'setuptools<80'
```

after ArduPilot prereqs and before `colcon build`.

Permanent rule:

- Pinning before an external prereq script is not enough.
- Pin again immediately before the build step that depends on the pin.

### M08 - Optional ROS Package Absence Was Not Treated Carefully Enough

What was missed:

- Some ROS packages can be absent from the current Ubuntu/ROS apt repository.

Observed behavior:

- `ros-humble-dvl-msgs` was not available during validation.

Root cause:

- Package availability differs by repository state and package source.

Fix applied:

- ROS package installation uses availability checks.
- Missing optional ROS packages produce warnings instead of aborting the entire
  install.

Permanent rule:

- Required packages may hard-fail.
- Optional packages must be checked with `apt_package_exists` or equivalent and
  must log a clear warning when unavailable.

### M09 - Existing ROS Apt Source Could Be Duplicated

What was missed:

- Ubuntu machines may already have ROS 2 configured through
  `/etc/apt/sources.list.d/ros2.sources`.

Observed failure class:

- Adding another `ros2.list` for the same repository can produce apt warnings or
  install instability.

Root cause:

- Clean VM logic assumed no ROS apt source existed.

Fix applied:

- Keep:

```bash
ensure_ros_apt_repo
cleanup_duplicate_ros_apt_repo
```

Permanent rule:

- Do not optimize away duplicate-source handling just because a fresh container
  works.

### M10 - Empty Shell Array Expansion Reached Python As A Blank Argument

What was missed:

- Shell arrays expanded with a default expression can pass an empty string.

Observed error:

```text
run_urdf_full.py: error: unrecognized arguments:
```

Root cause:

- An optional array was expanded like:

```bash
"${EXTRA_ARGS[@]-}"
```

- For an empty optional array, that can forward a blank argument to Python.

Fix applied:

- `uuv_mujoco/v2.2/launch_uuv_sim.sh` builds the command as an array and appends
  optional args only when `#EXTRA_ARGS > 0`.
- `uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh` constructs the SITL command with
  explicit array-length checks for `SIM_ARGS` and `USER_ARGS`.

Permanent rule:

- This search must return no matches:

```bash
rg -n '\$\{[A-Za-z0-9_]+\[@\]-\}' dist2/ubuntu22.04/*.sh uuv_mujoco/v2.2/*.sh document/docsource/*.sh
```

### M11 - Headless Smoke Test Needed A Different Success Definition

What was missed:

- The simulator is a long-running process.
- `timeout 90s ...` returning `124` can mean the simulator reached the running
  state and was killed by timeout, not that launch failed.

Observed behavior:

- A successful patched smoke test ended with timeout status `124`.
- The log showed headless mode, profile loading, thruster setup, and physics
  setup before timeout.

Root cause:

- Treating every non-zero timeout status as failure would misclassify a healthy
  long-running simulator.

Fix applied:

- Docker guide accepts exit `0` or `124`.
- Smoke criteria require checking log markers, not just the exit code.

Permanent rule:

- `124` is acceptable only when the log proves the simulator reached runtime.
- Immediate argparse/import/path errors are still failures.

### M12 - Docker Heredoc Validation Needs `docker run -i`

What was missed:

- Docker validation scripts passed through heredocs require stdin to stay
  attached.

Observed failure class:

- Inner install scripts can be skipped, truncated, or hidden behind shell
  quoting problems when stdin is not attached correctly.

Root cause:

- Long nested one-line Docker commands are fragile and hide the real installer
  error.

Fix applied:

- The guide uses:

```bash
docker run -i ... ubuntu:22.04 bash -s <<'ROOT_SCRIPT'
```

Permanent rule:

- Keep `-i` when using heredocs with Docker.
- Prefer heredocs over nested quoted one-liners for install validation.

### M13 - QGroundControl Was Not Truly Validated In ARM64 Docker

What was missed:

- Docker on Apple Silicon pulled Ubuntu 22.04 ARM64.
- The QGroundControl AppImage downloaded by the installer is x86_64.

Observed limitation:

- ARM64 Docker can validate apt, Python, ROS, MuJoCo imports, and headless
  simulator startup.
- It cannot prove the x86_64 QGroundControl GUI works.

Root cause:

- Architecture-specific GUI binary was tested in a non-matching architecture
  environment.

Fix applied:

- The guide marks QGC/GUI as requiring real x86_64 Ubuntu or explicit
  `--platform linux/amd64` validation.

Permanent rule:

- Do not claim GUI/QGC release success from ARM64 Docker alone.

### M14 - Host-Specific MuJoCo Asset Path Leaked Into Runtime

What was missed:

- A MuJoCo Ping360 asset config referenced a developer-machine path.

Affected file:

```text
uuv_mujoco/v2.2/assets/ping360/ping360_body_no_cable.json
```

Root cause:

- The JSON pointed at a path valid on the development Mac, not at a bundled
  asset path.

Fix applied:

- Use the packaged relative STL path:

```text
PING360_SONAR_BR-100399/PING360-ASM.STL
```

Permanent rule:

- Every JSON/XML mesh reference in `uuv_mujoco/v2.2/assets/**` must resolve
  inside the extracted package.

### M15 - ROS/RViz Files Contained Developer-Machine Paths

What was missed:

- ROS helper files under `rospkg/kmu26_auv` could contain absolute local paths.

Affected examples:

```text
rospkg/kmu26_auv/urdf/rov_rviz_local.urdf
rospkg/kmu26_auv/rviz/nuc_live.rviz
```

Root cause:

- URDF/RViz config was created from a local development environment and kept
  absolute file references.

Fix applied:

- URDF must avoid `file:///Users/...`.
- RViz must not point to an absolute local robot description file.

Permanent rule:

- RViz and URDF content must be portable by construction:
  package-relative path, ROS package URL, topic, or empty/default description.

### M16 - Documentation Helper Scripts Assumed The Original Checkout Path

What was missed:

- Several `document/docsource` helper scripts were tied to the development
  checkout path.

Observed failure class:

- Scripts run from the original Mac path but fail after extraction or on another
  Ubuntu machine.

Root cause:

- Hard-coded repository roots such as `/Users/kanghyunmin/...`.

Fix applied:

- Scripts derive root from `BASH_SOURCE[0]` or Python `Path(__file__)`.

Permanent rule:

- Any script included in the dist2 package must run from the extracted package,
  not from the original repository path.

### M17 - Real Rosbag Data Was Assumed But Not Bundled

What was missed:

- Some replay workflows assumed real April 1 rosbag data existed locally.

Observed failure class:

- Replay scripts fail on another computer because large real-world bag data is
  intentionally excluded from the release package.

Root cause:

- Data-heavy local experiment paths were treated like package content.

Fix applied:

- `document/docsource/run_closed_loop_april1_replay.sh` documents external bag
  requirements and accepts an explicit `--bag` path.

Permanent rule:

- If data is not bundled, scripts must require or document a path to that data.
- Never silently assume lab data exists under the extracted package.

### M18 - Nested Zips Could Still Carry Backup Or Generated Files

What was missed:

- Excluding junk from the top-level package is not enough.
- `uuv_mujoco.zip` and `rospkg/kmu26_auv.zip` can independently include stale
  backups, caches, `.git`, compiled Python, logs, or generated build outputs.

Observed failure class:

- A top-level archive appears clean while nested runtime archives still contain
  local artifacts.

Root cause:

- Packaging filters were not consistently applied recursively.

Fix applied:

- `package_dist2.sh` excludes backup/cache/generated patterns from nested zips.
- `verify_package.sh` checks nested zip contents.

Permanent rule:

- Always inspect nested zips, not only the top-level archive.

### M19 - Relative Package Output Paths Can Point At The Wrong Place

What was missed:

- Packaging scripts change directories internally.
- A relative output directory can be interpreted from an unexpected working
  directory.

Observed failure class:

- The package command appears to run, but later verification reads a different
  or stale zip.

Root cause:

- Relative `--out-dir` plus internal `cd` behavior.

Fix applied:

- Use absolute output paths for local validation:

```bash
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_test_out
```

Permanent rule:

- The zip path passed to `verify_package.sh` must be the exact file that will be
  shared.

### M20 - `dist2/` Is Ignored In This Workspace

What was missed:

- In this workspace, `git status` reports `dist2/` as ignored.
- `git diff -- dist2/ubuntu22.04/DIST_GUIDE.md` can show nothing even after the
  guide is edited.

Observed behavior:

```text
!! dist2/
```

Root cause:

- The local git ignore rules hide the distribution directory from normal git
  diff/status review.

Fix applied:

- Verify the file directly with `sed`, `rg`, `python3`, and by extracting the
  built zip.
- Rebuild the package and confirm the embedded `DIST_GUIDE.md` contains the new
  sections.

Permanent rule:

- Do not trust plain `git diff` to prove `dist2` documentation or package files
  changed in this workspace.
- If dist2 files must be versioned, force-add intentionally or move the source
  of truth to a tracked location.
- For release validation, inspect the generated zip contents directly.

### M21 - Hard-Coding The Current Zip Hash Inside The Packaged Guide Is Self-Referential

What was missed:

- `DIST_GUIDE.md` is included inside the zip.
- Writing the current zip hash inside `DIST_GUIDE.md` changes the guide, which
  changes the zip, which changes the hash again.

Observed behavior:

- After inserting a concrete zip SHA256 into this guide, rebuilding the zip
  produced a different SHA256.

Root cause:

- The artifact hash cannot be stable if the artifact embeds its own current
  hash as mutable content.

Fix applied:

- The guide points to `dist2/ubuntu22.04/out/SHA256SUMS` instead of embedding
  the current package hash.

Permanent rule:

- Store current artifact hashes in `SHA256SUMS` next to the zip.
- Do not put the current package hash inside a file that is itself packaged into
  that zip.

## Findings From Dist2 Repair Work

Record of concrete findings that must influence future releases.

### 2026-05-08 Native Python Was The Working Install Path

The known-good zip installed successfully because it used Ubuntu's native
Python by default and installed runtime packages into the user's site-packages.
The broken behavior appeared when the installer drifted toward a virtualenv-first
model. Virtualenv support can remain available, but the default release path
must stay native until a clean Ubuntu validation proves otherwise.

Required behavior:

- `PYTHON_MODE="${PYTHON_MODE:-native}"`
- default `MJ311_PYTHON=/usr/bin/python3`
- `pip --user` for native runtime packages
- no unconditional `MJ311_ROOT`
- no unconditional `MJ311_MJPYTHON`

### 2026-05-08 User Site Packages Must Not Be Blocked

`PYTHONNOUSERSITE=1` breaks the native install model because user-site packages
installed with `pip --user` become invisible to GUI and child runtime processes.
Release payloads must not contain `PYTHONNOUSERSITE=1` unless the entire Python
installation strategy is redesigned and revalidated.

The verifier must reject this string in packaged runtime files.

### 2026-05-08 Existing ROS Apt Sources Are Normal

Ubuntu machines may already have ROS configured with `ros2.sources`. The
installer must reuse the existing ROS apt source and must remove duplicate
`ros2.list` entries when needed. A clean VM is not enough proof here because
developer and lab machines often have ROS installed before this package.

### 2026-05-08 Optional ROS Packages Must Stay Optional

During validation, `ros-humble-dvl-msgs` was not available from the configured
Ubuntu/ROS apt repositories. The installer must not abort because one optional
ROS package is absent. Use availability checks, install packages that exist, and
log clear warnings for missing optional packages.

Important correction from the Docker install test: the apt package is optional,
but the message interface itself is not optional because `hit25_auv_ros2`
contains `find_package(dvl_msgs REQUIRED)` and `dvl_to_twist_bridge.cpp`
includes `<dvl_msgs/msg/dvl.hpp>`. Therefore the distribution must carry
`rospkg/dvl_msgs.zip` and build it locally whenever the apt package is absent.

### 2026-05-08 ArduPilot Prereqs Cannot Run As Root

The first Docker attempt failed because ArduPilot's upstream prereq script
refused to run as UID `0`. A Docker validation that runs the installer directly
as root is invalid. Create a normal user, grant passwordless sudo, and run the
installer from that user account.

### 2026-05-08 `SUDO_USER` Can Be Misleading In Docker

When the container used `sudo -Hiu uuv`, the effective user was `uuv` but
`SUDO_USER` could still be `root`. Any group-add logic must avoid adding `root`
to `dialout` by accident. Use `SUDO_USER` only when it is non-empty and not
`root`; otherwise use `id -un`.

### 2026-05-08 ArduPilot Can Disturb Python Tooling

ArduPilot's prereq script may install or upgrade Python packages used by build
tooling. In validation it caused setuptools to become newer than the version
known to work cleanly with the ROS/colcon setup. The installer must re-run:

```bash
python3 -m pip install --user 'setuptools<80'
```

after ArduPilot prereqs and before `colcon build`.

### 2026-05-08 Blank Shell Array Expansion Breaks Python CLIs

The headless smoke test initially failed with:

```text
run_urdf_full.py: error: unrecognized arguments:
```

The cause was expanding an empty array with a default expression such as
`"${EXTRA_ARGS[@]-}"`. That forwards an empty string to Python. Future shell
entrypoints must build command arrays explicitly and append optional arrays only
when their length is greater than zero.

Files that required this rule during repair:

- `uuv_mujoco/v2.2/launch_uuv_sim.sh`
- `uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh`

### 2026-05-08 Host Paths Must Be Treated As Release Blockers

Host-specific paths were found in runtime assets and ROS helper files. These are
release blockers because the package must run after extraction on another
machine.

Examples that must stay fixed:

- Ping360 MuJoCo asset JSON must use a relative STL path.
- RViz and URDF files must not reference `/Users/kanghyunmin/...`.
- Document helper scripts must derive their root from their own path.
- Replay scripts must not assume local rosbag paths from the development
  machine.

The verifier must scan both the top-level package and nested zips for
developer-machine paths.

### 2026-05-08 Generated And Backup Files Must Be Excluded Recursively

It is not enough to exclude backups from the top-level zip. Nested zips can
still accidentally carry `*.bak`, `*.bak_*`, `*~`, `*.orig`, cache directories,
compiled Python, logs, or `.git` content. `package_dist2.sh` and
`verify_package.sh` must enforce these rules recursively.

### 2026-05-08 Dist2 Does Not Bundle Every Runtime External

The package intentionally does not bundle ArduPilot, QGroundControl binaries, or
large real-world rosbag datasets. The installer downloads or clones external
software where appropriate. Data-heavy replay workflows must document their
external input paths clearly.

This is a release rule: do not add silent assumptions that those externals
already exist on the target machine.

## What Requires A New Ubuntu Install Test

Run a fresh Ubuntu 22.04 install test again after changing any of these:

- `install_uuv_sim_ubuntu22.sh`
- `.uuv_mujoco_env.sh` generation
- Python dependency list or MuJoCo version
- ROS apt setup, ROS package list, or colcon build logic
- ArduPilot clone/prereq/setup logic
- QGroundControl download or launch logic
- `run_control_gui.sh` import/environment behavior
- any script that launches MuJoCo, SITL, ROS, MAVProxy, or QGroundControl
- package include/exclude rules
- path resolution logic for assets, URDF, RViz, rosbag replay, or document
  helper scripts

Static verification is enough only for comment-only or documentation-only
changes that do not alter package contents used at install/runtime. If the
changed documentation is included in the zip, rebuild and re-run
`verify_package.sh` so the final artifact and `SHA256SUMS` are current.

## Failure Signature Table

Use the first real error in the log, not the final cascade.

| Error or symptom | Most likely cause | Required fix |
| --- | --- | --- |
| `Please do not run this script as root; don't sudo it!` | ArduPilot prereq script ran as UID `0`. | Run installer as a normal user with sudo access; Docker must create and switch to a non-root user. |
| `Could not find a package configuration file provided by "dvl_msgs"` | `ros-humble-dvl-msgs` was unavailable and the dist did not bundle/build `dvl_msgs`. | Include `rospkg/dvl_msgs.zip`, extract it before `kmu26_auv`, and build `dvl_msgs` in the same colcon workspace. |
| `run_urdf_full.py: error: unrecognized arguments:` | Empty optional shell array forwarded as a blank CLI arg. | Replace `"${ARRAY[@]-}"` with command-array construction and length checks. |
| `ModuleNotFoundError: No module named 'mujoco'` after install | Wrong interpreter, user-site blocked, or env not sourced. | Source `.uuv_mujoco_env.sh`; verify native `/usr/bin/python3`; remove `PYTHONNOUSERSITE=1`; reinstall pip deps with `--user`. |
| `MJ311_MJPYTHON` path not executable | Env file exported non-existent `mjpython`. | In native mode unset it; in venv mode export only when executable exists. |
| apt warns about duplicate ROS sources | Installer added `ros2.list` despite existing `ros2.sources`. | Keep `ensure_ros_apt_repo` and `cleanup_duplicate_ros_apt_repo`. |
| apt cannot locate one ROS helper package | Optional ROS package missing from repo. | Use `apt_package_exists`; warn and continue for optional packages. |
| `colcon build` fails or warns after ArduPilot prereqs | ArduPilot prereqs changed Python build tooling. | Re-run `python3 -m pip install --user 'setuptools<80'` before colcon. |
| Runtime tries to open `/Users/kanghyunmin/...` | Host-specific path leaked into package. | Rewrite to relative path, package URL, script-derived root, or explicit env/CLI path. |
| RViz opens without robot model | RViz config points to an absolute local URDF or wrong description source. | Use topic/default description settings or package-relative paths. |
| MuJoCo asset load fails for Ping360 STL | JSON/XML mesh path points outside bundled assets. | Use bundled relative path such as `PING360_SONAR_BR-100399/PING360-ASM.STL`. |
| Replay script cannot find a real bag file | Real rosbag data is intentionally not bundled. | Require `--bag /path/to/file` and document data acquisition separately. |
| QGroundControl AppImage does not run in ARM64 Docker | The downloaded AppImage is x86_64. | Validate QGC on x86_64 Ubuntu or an explicit amd64 environment. |
| `verify_package.sh` passes but Ubuntu install fails | Verifier only checked structure/syntax. | Run the Docker or real Ubuntu install test and add a new rule for the missed failure. |
| Zip contains files that were already fixed in source | Edited staged output or verified a stale artifact. | Clean/rebuild package and verify the exact zip to upload. |

## Comparing A Future Zip Against The Reference

Extract both packages:

```bash
rm -rf /tmp/uuv_ref /tmp/uuv_new
mkdir -p /tmp/uuv_ref /tmp/uuv_new
unzip -q /path/to/reference/uuv_sim_ubuntu22.04_dist2.zip -d /tmp/uuv_ref
unzip -q /path/to/new/uuv_sim_ubuntu22.04_dist2.zip -d /tmp/uuv_new
diff -qr /tmp/uuv_ref/uuv_sim_ubuntu22.04_dist2 /tmp/uuv_new/uuv_sim_ubuntu22.04_dist2
```

For installer review:

```bash
diff -u \
  /tmp/uuv_ref/uuv_sim_ubuntu22.04_dist2/install_uuv_sim_ubuntu22.sh \
  /tmp/uuv_new/uuv_sim_ubuntu22.04_dist2/install_uuv_sim_ubuntu22.sh
```

Focus on installer drift first. Runtime zip differences can affect simulation
behavior, but they should not usually prevent the install from completing.

## Common Failure Causes

Check these first if Ubuntu installation fails:

1. The installer default changed from native Python to venv.
2. `MJ311_MJPYTHON` is exported to a path that does not exist.
3. Existing ROS apt sources are duplicated instead of reused.
4. `colcon build` is running with incompatible setuptools.
5. ROS setup is sourced under `set -u`.
6. Optional apt packages are installed without availability checks.
7. `mujoco` version is unpinned and a newer pip release changed behavior.
8. ArduPilot prereq script changed the shell environment unexpectedly.
9. User-site pip packages are blocked by a system Python policy.
10. `ModemManager` grabs serial devices during QGroundControl or MAVLink tests.
11. The installer is run as root, causing ArduPilot prereqs to abort.
12. Docker group/user logic uses `SUDO_USER=root` instead of the actual install
    user.
13. A shell entrypoint forwards an empty string from an optional array to
    Python.
14. Nested zips still contain backup files, cache files, `.git`, or generated
    build outputs.
15. Runtime assets contain absolute host paths from the development Mac.
16. QGroundControl is tested in ARM64 Docker even though the AppImage is x86_64.
17. A replay script assumes a real rosbag exists even though large bag data is
    intentionally not bundled.
18. Package output and verifier input point to different zip files because a
    relative `--out-dir` was interpreted from a changed working directory.
19. `dvl_msgs` is treated as an apt-only dependency even though Humble apt did
    not provide `ros-humble-dvl-msgs` during validation.

When debugging, capture the exact failing command and the 30-50 log lines above
the first error. Later cascading errors are usually less useful.
