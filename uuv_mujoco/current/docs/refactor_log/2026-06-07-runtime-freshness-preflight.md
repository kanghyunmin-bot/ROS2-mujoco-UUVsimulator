# Runtime freshness preflight

Date: 2026-06-07

## Scope

- Added `tools/check_runtime_freshness.py`.
- Wired root launch wrappers through the freshness check:
  - `uuv_mujoco/run_mujoco.sh`
  - `uuv_mujoco/start_sitl_mujoco.sh`
  - `uuv_mujoco/start_docker_sitl_mujoco.sh`
  - `uuv_mujoco/reset_sim.sh`
- Updated the source-contract audit to require the checker and launcher calls
  as active-runtime evidence.

## Contract

The active runtime is still `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.  The directory name is not a release freshness
signal.  Freshness is now checked by comparing local `HEAD` with
`origin/uuv_sim` and by validating `uuv_mujoco/RUNTIME_VERSION.json`.

## Verification

```text
python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch
```

Result:

```text
[uuv_mujoco] runtime freshness: PASS
[uuv_mujoco] uuv_mujoco/current -> v2.2
[uuv_mujoco] branch=uuv_sim
[uuv_mujoco] HEAD=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
[uuv_mujoco] origin/uuv_sim=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
```

Shell syntax checks passed for all four root wrappers.
