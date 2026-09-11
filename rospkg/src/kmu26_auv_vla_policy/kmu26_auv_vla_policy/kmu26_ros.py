"""ROS sensor-to-HTTP-to-RC adapter; explicit enable plus live deadman required.

Run with ROS Humble Python and the updated collector package sourced. This
module deliberately does not import torch, GR00T models or GPU dependencies.
"""

import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from kmu26_auv_vla_data_collector.collector import VlaDataCollector
from mavros_msgs.msg import OverrideRCIn, State
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .kmu26_contract import CommandLimiter, motion_chunk, release_channels


def request_action(url, observation, timeout):
    """Use the server's explicit encoded payload format, with bounded HTTP waits."""
    import json_numpy
    import requests

    response = requests.post(
        url.rstrip("/") + "/act",
        json={"encoded": json_numpy.dumps({"observation": observation})},
        timeout=(timeout, timeout),
    )
    response.raise_for_status()
    return motion_chunk(json_numpy.loads(response.text))


class RovPolicyAdapter(Node):
    def __init__(self, sensors):
        super().__init__("vla_policy")
        self.sensors = sensors
        self.url = str(
            self.declare_parameter("server_url", "http://127.0.0.1:8000").value
        )
        self.dry_run = bool(self.declare_parameter("dry_run", True).value)
        self.expected_mode = str(
            self.declare_parameter("expected_mode", "STABILIZE").value
        )
        self.timeout = float(self.declare_parameter("max_action_age_sec", 0.3).value)
        if not np.isfinite(self.timeout) or not 0.1 <= self.timeout <= 1.0:
            raise ValueError("max_action_age_sec must be in [0.1, 1.0]")
        self.limiter = CommandLimiter(
            limit=float(self.declare_parameter("command_limit", 0.3).value),
            slew_per_second=float(
                self.declare_parameter("command_slew_per_sec", 0.5).value
            ),
            neutral=sensors._neutral_pwm,
            span=sensors._pwm_span,
        )
        if sensors._action_channel_indices != (4, 5, 2, 3):
            raise ValueError("This adapter requires ArduSub channels [5, 6, 3, 4]")
        self.output = self.create_publisher(
            OverrideRCIn,
            "/vla/proposed_rc" if self.dry_run else "/mavros/rc/override",
            10,
        )
        self.create_subscription(Bool, "/vla/deadman", self._deadman, 10)
        self.create_subscription(
            State, "/mavros/state", self._state, qos_profile_sensor_data
        )
        self.create_service(SetBool, "/vla/enable", self._enable)
        self.enabled = False
        self.owned = False
        self.deadman_at = -float("inf")
        self.state_at = -float("inf")
        self.vehicle_state = None
        self.epoch = 0
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.chunk = None
        self.requested_at = 0.0
        self.chunk_at = 0.0
        self.requested_ros_at = 0.0
        self.chunk_ros_at = 0.0
        self.last_command_ros_at = None
        self.last_tick = time.monotonic()
        self.last_ros_time = sensors._now()
        self.clock_advanced_at = self.last_tick
        self.timer = self.create_timer(
            0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            f"VLA adapter disabled, dry_run={self.dry_run}; no arm/mode changes"
        )

    def _deadman(self, message):
        if message.data:
            self.deadman_at = time.monotonic()
        else:
            self.deadman_at = -float("inf")
            self._stop()

    def _state(self, message):
        self.vehicle_state = message
        self.state_at = time.monotonic()

    def _ready(self, now):
        if now - self.deadman_at > 0.3:
            return False
        if self.dry_run:
            return True
        state = self.vehicle_state
        return bool(
            state
            and now - self.state_at <= 1.0
            and state.connected
            and state.armed
            and state.mode == self.expected_mode
            and self.count_publishers(self.output.topic_name) == 1
        )

    def _enable(self, request, response):
        self._stop()
        if not request.data:
            response.success = True
            response.message = "VLA disabled and owned channels released"
            return response
        try:
            if not self._ready(time.monotonic()):
                raise ValueError(
                    "Live deadman and (for RC output) armed, connected expected mode and exclusive ROS RC publisher required"
                )
            self.sensors.policy_observation(np.zeros(4))
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        self.enabled = True
        response.success = True
        response.message = "Enabled; keep /vla/deadman true at 10 Hz"
        return response

    def _publish(self, channels):
        message = OverrideRCIn()
        message.channels = channels
        self.output.publish(message)

    def _stop(self):
        self.enabled = False
        self.epoch += 1
        self.chunk = None
        self.limiter.reset()
        self.last_command_ros_at = None
        if self.owned:
            self._publish(release_channels())
            self.owned = False

    def _tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        ros_time = self.sensors._now()
        if ros_time > self.last_ros_time:
            self.clock_advanced_at = now
        reset = ros_time < self.last_ros_time
        self.last_ros_time = ros_time
        if not self.enabled:
            return
        if (
            reset
            or now - self.clock_advanced_at > self.timeout
            or not self._ready(now)
            or not 0 < dt <= 0.2
        ):
            self._stop()
            return
        try:
            observation = self.sensors.policy_observation(self.limiter.previous)
            if self.future is not None and self.future.done():
                future, self.future = self.future, None
                result = future.result()
                if (
                    self.request_epoch == self.epoch
                    and now - self.requested_at <= self.timeout
                ):
                    self.chunk = result
                    self.chunk_at = self.requested_at
                    self.chunk_ros_at = self.requested_ros_at
            if self.chunk is not None:
                age = now - self.chunk_at
                if age > self.timeout:
                    self._stop()
                    return
                # Training cadence is ROS time. Wall time remains the safety watchdog.
                ros_age = ros_time - self.chunk_ros_at
                if not 0 <= ros_age < 1.6:
                    self._stop()
                    return
                command_dt = (
                    0.1
                    if self.last_command_ros_at is None
                    else ros_time - self.last_command_ros_at
                )
                if command_dt >= 0.1 - 1e-6:
                    if command_dt > 0.2:
                        self._stop()
                        return
                    action = self.chunk[min(int((ros_age + 1e-6) * 10), 15)]
                    self._publish(self.limiter.apply(action, command_dt))
                    self.last_command_ros_at = ros_time
                    self.owned = True
            if self.future is None:
                self.requested_at = now
                self.requested_ros_at = ros_time
                self.request_epoch = self.epoch
                self.future = self.pool.submit(
                    request_action, self.url, observation, self.timeout
                )
            elif now - self.requested_at > self.timeout:
                self._stop()
        except Exception as error:
            self.get_logger().warning(f"VLA stopped: {error}")
            self._stop()

    def close(self):
        self._stop()
        self.pool.shutdown(wait=False, cancel_futures=True)


def main():
    rclpy.init()
    sensors = VlaDataCollector()
    adapter = RovPolicyAdapter(sensors)
    executor = SingleThreadedExecutor()
    executor.add_node(sensors)
    executor.add_node(adapter)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.close()
        executor.shutdown()
        adapter.destroy_node()
        sensors.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
