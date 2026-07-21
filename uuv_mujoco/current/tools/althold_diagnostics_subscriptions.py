"""ROS subscription wiring for the ALT_HOLD diagnostics logger."""

from __future__ import annotations


def create_subscriptions(owner, *, queue_size: int = 20) -> None:
    node = owner.node
    q = int(queue_size)
    sensor_qos = owner.sensor_qos
    node.create_subscription(owner.ManualControl, "/mavros/manual_control/send", owner._on_manual, q)
    node.create_subscription(owner.RCIn, "/mavros/rc/in", owner._on_rc_in, q)
    node.create_subscription(owner.RCOut, "/mavros/rc/out", owner._on_rc_out, q)
    node.create_subscription(owner.Float32, "/depth", owner._on_depth, sensor_qos)
    node.create_subscription(owner.Float32, "/bar30/pressure_pa", owner._on_pressure, sensor_qos)
    node.create_subscription(
        owner.PoseStamped,
        "/mavros/local_position/pose",
        owner._on_mavros_pose,
        sensor_qos,
    )
    node.create_subscription(
        owner.TwistStamped,
        "/mavros/local_position/velocity_local",
        owner._on_mavros_vel,
        sensor_qos,
    )
    node.create_subscription(owner.TwistStamped, "/dvl/velocity", owner._on_dvl_vel, sensor_qos)
    node.create_subscription(
        owner.PoseStamped,
        "/mujoco/ground_truth/pose",
        owner._on_mujoco_pose,
        sensor_qos,
    )
    node.create_subscription(owner.Odometry, "/sim/odom", owner._on_sim_odom, sensor_qos)
    node.create_subscription(
        owner.String,
        "/uuv_mujoco/sitl/mavlink_telemetry_status",
        owner._on_sitl_mavlink_status,
        q,
    )
    node.create_subscription(
        owner.String,
        "/uuv_mujoco/sitl/sensor_replay_status",
        owner._on_sitl_sensor_status,
        q,
    )
    node.create_timer(1.0 / max(owner.sample_hz, 1.0), owner._sample)
