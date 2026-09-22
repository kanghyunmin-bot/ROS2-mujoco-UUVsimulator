"""Install opt-in SITL yaw braking without changing hardware builds."""

import argparse
from pathlib import Path
import subprocess


def main() -> int:
    """Check or apply the version-controlled patch to the selected source tree."""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ardupilot_dir", type=Path, default=root / "ardupilot_sub_stable"
    )
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    patch = root / "setup/patches/ardusub-sitl-yaw-braking.patch"
    command = ["git", "-C", str(args.ardupilot_dir), "apply"]
    installed = subprocess.run(
        command + ["--reverse", "--check", str(patch)], capture_output=True
    )
    if installed.returncode == 0:
        print("SITL yaw braking source already installed; rebuild to update binary.")
        return 0
    subprocess.run(command + ["--check", str(patch)], check=True)
    if not args.apply:
        print("Patch required: rerun with --apply, then rebuild SITL.")
        return 1
    subprocess.run(command + [str(patch)], check=True)
    print("Applied SITL-only yaw braking; rebuild before selecting the stable preset.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
