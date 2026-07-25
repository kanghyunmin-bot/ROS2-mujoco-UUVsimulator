# ROS2 Endpoints and Hydrostatic Runtime Split

Date: 2026-06-07

Scope: active runtime under `sim/current`.

## Changed

- Split `bridge/ros2_endpoints.py` into focused endpoint builders:
  - `bridge/ros2_endpoint_publishers.py`
  - `bridge/ros2_endpoint_subscriptions.py`
  - `bridge/ros2_endpoint_services.py`
- Kept `create_ros2_endpoints()` as the public facade used by
  `bridge/ros2_bridge_init.py`.
- Split `sim/physics/hydrostatic_setup.py` into focused hydrostatic runtime
  helpers:
  - `sim/physics/hydrostatic_runtime_types.py`
  - `sim/physics/hydrostatic_runtime_sources.py`
  - `sim/physics/hydrostatic_runtime_values.py`
  - `sim/physics/hydrostatic_runtime_reporting.py`

## Contract Preserved

- ROS2 topic, subscription, and service names are unchanged.
- MAVROS surface enabled/disabled behavior is unchanged.
- Optional `RCOut`, `SonarEcho`, `DVLMsg`, and `DVLDRMsg` handling is unchanged.
- Hydrostatic env/profile value names are unchanged.
- CoB runtime override logging, CoB site alignment, hydrostatic source
  fallback, restoring trim, and real-start trim blending are unchanged.

## Validation

- `python3 -m compileall -q sim/current/bridge/ros2_endpoints.py sim/current/bridge/ros2_endpoint_publishers.py sim/current/bridge/ros2_endpoint_subscriptions.py sim/current/bridge/ros2_endpoint_services.py`
- Fake-node endpoint construction:
  - MAVROS enabled: `pubs=40`, `subs=7`, `srvs=3`
  - MAVROS disabled: `pubs=26`, `subs=3`, `srvs=0`
- `python3 -m compileall -q sim/current/sim/physics/hydrostatic_setup.py sim/current/sim/physics/hydrostatic_runtime_types.py sim/current/sim/physics/hydrostatic_runtime_sources.py sim/current/sim/physics/hydrostatic_runtime_values.py sim/current/sim/physics/hydrostatic_runtime_reporting.py`
- `/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_hydrostatic_split --simulate-s 0`

## Results

- ROS2 endpoint facade is removed from the top hotspot list.
- Static physics audit still reports:
  - `mass=15.000 kg`
  - `neutral_volume=0.015000 m^3`
  - `buoyancy_scale=1.000000`
  - `source=body_components`
  - `net_down=+0.000N`
  - `required_scale=1.000000`
