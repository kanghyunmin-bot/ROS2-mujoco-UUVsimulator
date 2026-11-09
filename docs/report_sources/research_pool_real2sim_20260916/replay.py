import bisect
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from auv_dvl_a50_msg.msg import DVL
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

R = Path(
    os.environ.get(
        "REPLAY_OUTPUT_ROOT", "/workspace/outputs/research-pool-real2sim-20260916"
    )
)
cfg = json.loads((R / (os.environ["BAG"] + "_commands.json")).read_text())
out = R / os.environ.get("CASE", "controller")
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node("isolated_real_command_replay")
clock = None
state = None
origin = None
done = False
pending = []
logs = []
published = []
tracks = {}
last_rc = -1
last_state = None
sensors = {}


def sensor_row(key, msg, values):
    if clock is not None:
        sensors.setdefault(key, []).append(
            [clock, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, *values]
        )


n.create_subscription(
    Imu,
    "/mavros/imu/data",
    lambda m: sensor_row(
        "imu",
        m,
        [
            m.orientation.x,
            m.orientation.y,
            m.orientation.z,
            m.orientation.w,
            m.angular_velocity.x,
            m.angular_velocity.y,
            m.angular_velocity.z,
        ],
    ),
    qos_profile_sensor_data,
)
n.create_subscription(
    PoseWithCovarianceStamped,
    "/depth/pose",
    lambda m: sensor_row("depth", m, [m.pose.pose.position.z]),
    qos_profile_sensor_data,
)
n.create_subscription(
    DVL,
    "/dvl/data",
    lambda m: sensor_row(
        "dvl",
        m,
        [m.velocity.x, m.velocity.y, m.velocity.z, m.altitude, float(m.velocity_valid)],
    ),
    qos_profile_sensor_data,
)
pub = n.create_publisher(OverrideRCIn, "/mavros/rc/override", 100)
arm = n.create_client(CommandBool, "/mavros/cmd/arming")
mode = n.create_client(SetMode, "/mavros/set_mode")
release = n.create_client(Trigger, "/mujoco/release_initial_depth_hold")


def on_clock(m):
    global clock
    clock = m.clock.sec + m.clock.nanosec / 1e9


def on_state(m):
    global state
    state = m
    logs.append(
        {
            "kind": "state",
            "clock": clock,
            "armed": m.armed,
            "mode": m.mode,
            "connected": m.connected,
        }
    )


def on_pose(topic, m):
    if clock is None:
        return
    pose = m.pose.pose if isinstance(m, Odometry) else m.pose
    tracks.setdefault(topic, []).append(
        [
            clock,
            m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )


n.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
n.create_subscription(State, "/mavros/state", on_state, 10)
for topic, typ in [
    ("/mujoco/ground_truth/pose", PoseStamped),
    ("/odometry/filtered", Odometry),
    ("/mavros/local_position/odom", Odometry),
]:
    n.create_subscription(
        typ, topic, lambda m, t=topic: on_pose(t, m), qos_profile_sensor_data
    )
n.create_subscription(
    RCOut,
    "/mavros/rc/out",
    lambda m: logs.append(
        {"kind": "rcout", "clock": clock, "channels": list(m.channels)}
    ),
    qos_profile_sensor_data,
)


def call(client, req, kind):
    f = client.call_async(req)
    pending.append((f, kind, clock))
    logs.append({"kind": "request", "request": kind, "clock": clock})


def spin_until(pred, limit):
    end = time.monotonic() + limit
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
        if pred():
            return True
    return False


def interrupted(*_):
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)
try:
    if not spin_until(
        lambda: (
            clock is not None
            and state is not None
            and state.connected
            and bool(tracks.get("/mujoco/ground_truth/pose"))
            and arm.service_is_ready()
            and mode.service_is_ready()
        ),
        60,
    ):
        raise RuntimeError("stack not ready")
    if not spin_until(
        lambda: "vehicle heartbeat state" in (out / "mujoco.log").read_text(), 15
    ):
        raise RuntimeError("MuJoCo heartbeat safety gate is not connected")
    call(mode, SetMode.Request(custom_mode="ALT_HOLD"), "initial_ALT_HOLD")
    if not spin_until(lambda: state.mode == "ALT_HOLD", 15):
        raise RuntimeError("ALT_HOLD not acknowledged")
    # Warm up FCU while depth hold pins the initialization; replay clock starts afterwards.
    t = clock
    spin_until(lambda: clock - t >= 5, 15)
    origin = clock
    start_wall = time.monotonic()
    deadline = start_wall + 600
    rc_times = [x[0] for x in cfg["rc"]]
    state_times = [x[0] for x in cfg["states"]]
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.002)
        elapsed = clock - origin + cfg["begin"]
        idx = bisect.bisect_right(rc_times, elapsed) - 1
        if idx >= 0 and idx != last_rc:
            first = last_rc + 1 if last_rc >= 0 else idx
            for due in range(first, idx + 1):
                msg = OverrideRCIn()
                msg.channels = cfg["rc"][due][1]
                pub.publish(msg)
                published.append([elapsed, rc_times[due], due, 0])
            last_rc = idx
        st = bisect.bisect_right(state_times, elapsed) - 1
        if st >= 0 and st != last_state:
            wanted = cfg["states"][st]
            call(mode, SetMode.Request(custom_mode=wanted[3]), "mode_" + wanted[3])
            call(arm, CommandBool.Request(value=wanted[2]), "arm_" + str(wanted[2]))
            last_state = st
            if wanted[2] and release.service_is_ready():
                call(release, Trigger.Request(), "release_initial_depth_hold")
        for f, kind, at in pending[:]:
            if f.done():
                logs.append(
                    {
                        "kind": "response",
                        "request": kind,
                        "requested_clock": at,
                        "clock": clock,
                        "result": str(f.result()),
                    }
                )
                pending.remove((f, kind, at))
        if elapsed >= cfg["end"]:
            done = True
            break
        if int(elapsed) % 10 == 0 and (not logs or logs[-1].get("kind") != "progress"):
            pass
    if not done:
        raise RuntimeError("replay timeout")
    print(
        "REPLAY COMPLETE",
        clock - origin,
        "sim seconds",
        time.monotonic() - start_wall,
        "wall seconds",
        len(published),
        "RC publishes",
        flush=True,
    )
finally:
    msg = OverrideRCIn()
    msg.channels = [65535] * 18
    for axis in (2, 3, 4, 5):
        msg.channels[axis] = 1500
    pub.publish(msg)
    if arm.service_is_ready():
        call(arm, CommandBool.Request(value=False), "final_disarm")
        spin_until(lambda: state is not None and not state.armed, 5)
    msg = OverrideRCIn()
    msg.channels = [65535] * 18
    for axis in (2, 3, 4, 5):
        msg.channels[axis] = 0
    pub.publish(msg)
    (out / "replay.json").write_text(
        json.dumps(
            {
                "complete": done,
                "origin_sim_time": origin,
                "begin_bag_time": cfg["begin"],
                "end_bag_time": cfg["end"],
                "published": published,
                "logs": logs,
                "tracks": tracks,
                "sensors": sensors,
                "wall_replay_s": time.monotonic() - start_wall
                if origin is not None
                else None,
            },
            indent=2,
        )
    )
    n.destroy_node()
    rclpy.shutdown()
