"""Restore the launched scene between demonstrations without rewinding ROS time."""
from copy import deepcopy
from queue import SimpleQueue, Empty
import time

from bridge.ros2_runtime_mavros_state_values import mavros_state_values
from bridge.ros2_mavros_arm_mode_transport import send_mode_to_sitl

from sim.runtime.underwater_relative_acceleration import reset_relative_acceleration_history


class DemoReset:
    """Physics-thread reset with a two-second sensor settling interval [s]."""

    def __init__(self, step, physics, bridge):
        self.step, self.physics, self.bridge = step, physics, bridge
        self.qpos = step.data.qpos.copy()
        self.eq_active = step.data.eq_active.copy()
        self.model_arrays = {name: getattr(step.model, name).copy() for name in
                             ('eq_data', 'geom_contype', 'geom_conaffinity', 'geom_rgba')}
        self.buoy_runtime = bridge._course_buoy_runtime
        self.buoys = deepcopy(self.buoy_runtime.buoys)
        self.requests = SimpleQueue()
        self.pending = None
        self.until = None

    def install(self):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.task import Future
        from std_srvs.srv import Trigger

        async def request(_request, response):
            future = Future()
            self.requests.put((time.monotonic(), future, response))
            return await future

        self.service = self.bridge.node.create_service(
            Trigger, '/uuv_mujoco/demo_reset', request,
            callback_group=ReentrantCallbackGroup())

    def restore(self):
        step, data = self.step, self.step.data
        data.qpos[:] = self.qpos
        for name in ('qvel', 'qacc', 'qacc_warmstart', 'ctrl', 'act', 'xfrc_applied', 'qfrc_applied'):
            getattr(data, name)[:] = 0
        data.eq_active[:] = self.eq_active
        for name, values in self.model_arrays.items():
            getattr(step.model, name)[:] = values
        self.buoy_runtime.buoys[:] = deepcopy(self.buoys)
        self.buoy_runtime._next_update_time_s = -1.0
        self.buoy_runtime._velocity_cache_active = False
        actuator = self.physics.thruster_actuator_runtime
        for name in ('state', 'target', 'force_cmd', 'prop_phase'):
            values = getattr(actuator, name)
            for key in values:
                values[key] = 0.0
        for name in ('last_reaction_torque_world', 'last_reaction_torque_body', 'last_force_body', 'last_torque_body'):
            getattr(actuator, name)[:] = 0.0
        reset_relative_acceleration_history(self.physics.underwater_wrench_runtime)
        for name in ('_sitl_prev_vel_sim_t', '_sitl_prev_vel_enu',
                     '_sitl_bar30_prev_depth_m', '_sitl_bar30_prev_t'):
            setattr(self.bridge, name, None)
        # Drop pre-reset delayed camera packets without resetting noise seeds.
        for camera in getattr(self.bridge, '_camera_sensor_runtimes', {}).values():
            camera.transport.discard_pending()
        step.mujoco.mj_forward(step.model, data)

    def tick(self, *, is_paused, publish_ros):
        """Consume requests and hold the restored pose while fresh sensors arrive."""
        while True:
            try:
                issued, future, response = self.requests.get_nowait()
            except Empty:
                break
            armed, mode, connected, _ = mavros_state_values(self.bridge)
            if is_paused or self.pending or time.monotonic() - issued > 3 or not connected or mode not in {'STABILIZE', 'MANUAL'}:
                response.success = False
                response.message = '일시정지를 해제하고 연결된 STABILIZE/MANUAL에서 초기화하세요. 중복·지연 요청은 거절됩니다.'
                future.set_result(response)
                continue
            if not send_mode_to_sitl(self.bridge, 'MANUAL'):
                response.success = False
                response.message = '초기화 준비 모드 전환이 거절되었습니다.'
                future.set_result(response)
                continue
            self.pending = (future, response)
            self.original_mode, self.original_armed = mode, armed
            self.mode_restoring = False
            self.deadline = time.monotonic() + 12.0
            self.until = float(self.step.data.time) + 2.0
            try:
                self.restore()
            except Exception as exc:
                self.finish(False, f'초기화 실패: {exc}')
                raise
        if self.pending is None:
            return False
        # Run on the physics thread, not on a ROS callback thread. Simulation
        # time remains monotonic; noise/drift RNGs and FCU ARM state continue.
        if not is_paused:
            self.step.data.time += float(self.step.model.opt.timestep)
        if publish_ros:
            self.step.publish_ros_once()
            self.step.publish_qgc_video_once()
        armed, mode, connected, _ = mavros_state_values(self.bridge)
        if not connected or armed != self.original_armed or time.monotonic() > self.deadline:
            self.finish(False, '초기화 완료를 확인하지 못했습니다. 연결·ARM·모드를 확인하세요. 자동 재시도하지 않습니다.')
        elif self.step.data.time >= self.until:
            transport = self.bridge._sitl_transport
            if transport is not None and not self.mode_restoring:
                if mode != 'MANUAL':
                    return True
                if not send_mode_to_sitl(self.bridge, self.original_mode):
                    self.finish(False, '원래 조종 모드 복귀가 거절되었습니다. 모드를 확인하세요.')
                    return True
                self.mode_restoring = True
                self.until = float(self.step.data.time) + 0.5
                return True
            if mode == self.original_mode:
                self.finish(True, '다음 시연 초기화 완료: 로봇·부표 복구, ARM·시뮬 시간·세션 유지. 조종 입력을 켜고 녹화하세요.')
        return True

    def finish(self, success, message):
        future, response = self.pending
        response.success, response.message = success, message
        self.pending = None
        future.set_result(response)
