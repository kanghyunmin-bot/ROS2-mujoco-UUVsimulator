"""Regression coverage for FCU cadence independent of ROS and GUI publication."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge.ros2_bridge_publish import publish
from bridge.ros2_sitl_sensor_types import Ros2SensorSnapshot
from gui.sim_stack_env_contract import build_gui_sim_stack_env
from sim.runtime.simulation_step_raw_pwm import run_raw_pwm_runtime_step
from test_imu_bar30_sensor_runtime import (
    FakeBridge, profile, base_state, imu_state, vertical_state,
    configure_imu_bar30_sensor_runtime, advance_imu_bar30_sensor_runtime,
)


class FcuCadenceTest(unittest.TestCase):
    def test_old_and_finer_steps_align_to_fcu_clock(self):
        from sim.runtime.model_runtime_setup import _align_fcu_timestep
        for requested, expected in ((.005, .0025), (.002, .00125), (.0025, .0025)):
            model = SimpleNamespace(opt=SimpleNamespace(timestep=requested))
            _align_fcu_timestep(model, 400.)
            self.assertAlmostEqual(model.opt.timestep, expected)

    def test_gui_profiles_cannot_slow_fcu_physics(self):
        for name in ('low', 'balanced', 'high'):
            env = build_gui_sim_stack_env({'UUV_RUNTIME_PROFILE': name}, backend='docker', sim_stack_dir=ROOT)
            self.assertEqual(float(env['UUV_MUJOCO_TIMESTEP']), .0025)
            self.assertGreaterEqual(float(env['ROS2_UUV_SITL_POLL_THREAD_HZ']), 200)
            self.assertEqual(env['SITL_SPEEDUP_DEFAULT'], '1')

    def test_fcu_runs_between_ros_publishes_and_keeps_deliveries(self):
        snapshot = Ros2SensorSnapshot(None, None, None, None, None, None, None, None, 0., 0.)
        owner = SimpleNamespace(enable_sitl=True, last_pub_t=-1., sensor_dt=.02,
                                _sitl_transport=None, _enable_ros=True, _ros_ok=True)
        data = SimpleNamespace(time=0.)
        sent = []
        def build(_data):
            sent.append(_data.time)
            return replace(snapshot, imu_sensor_deliveries=(_data.time,))
        owner._build_and_send_sitl_sensor_snapshot = build
        with patch('bridge.ros2_bridge_publish.publish_ros_snapshot') as ros:
            for i in range(9):
                data.time = i * .0025
                publish(owner, data)
            self.assertEqual(len(sent), 9)
            self.assertEqual(ros.call_count, 9)
            self.assertEqual(sum(call.kwargs['general_due'] for call in ros.call_args_list), 2)
            self.assertEqual([call.args[3].imu_sensor_deliveries for call in ros.call_args_list],
                             [(i * .0025,) for i in range(9)])
            publish(owner, data)
            self.assertEqual(len(sent), 9, 'paused time must not create another capture')
            data.time = 0.
            publish(owner, data)
            self.assertEqual(ros.call_args.args[3].imu_sensor_deliveries, (0.,))

    def test_servo_consumed_before_thruster_target_update(self):
        from threading import Lock
        from sim.runtime.simulation_step_runtime import SimulationStepRuntime
        events = []
        transport = SimpleNamespace(_poll_servo_endpoint=lambda: events.append('servo'))
        bridge = SimpleNamespace(_sitl_transport=transport, _sitl_transport_lock=Lock())
        def due():
            events.append('target')
            return True, .0025
        owner = SimpleNamespace(spin_ros_once=Mock(), sitl_enabled=True, raw_pwm_mode=True,
                                get_ros_bridge=lambda: bridge, thruster_update_due=due,
                                run_raw_pwm_step=Mock(return_value=(0, 0, 0, 0)))
        with patch('sim.runtime.simulation_step_runtime.record_step_phase'):
            SimulationStepRuntime.run_step(owner, False)
        self.assertEqual(events, ['servo', 'target'])

    def test_sensor_arrivals_bypass_general_telemetry_gate(self):
        from bridge.ros2_bridge_publish_ros import publish_ros_snapshot
        owner = SimpleNamespace(_enable_ros=True, _ros_ok=True,
                                _publish_static_context=Mock(return_value=True),
                                _flush_ros_publish_jobs=Mock())
        state = object()
        with patch('bridge.ros2_bridge_publish_ros.prepare_ros_publish_state', return_value=state), \
             patch('bridge.ros2_bridge_publish_ros.acquire_ros_stamp', return_value=object()), \
             patch('bridge.ros2_bridge_publish_ros.flush_sensor_packet_jobs') as packets:
            publish_ros_snapshot(owner, object(), .0025, object(), general_due=False)
            packets.assert_called_once()
            owner._flush_ros_publish_jobs.assert_not_called()
            publish_ros_snapshot(owner, object(), .02, object(), general_due=True)
            self.assertEqual(packets.call_count, 1, 'general tick must not duplicate arrivals')
            owner._flush_ros_publish_jobs.assert_called_once()

    def test_strict_packet_flush_preserves_capture_and_ahrs_ownership(self):
        from test_imu_bar30_publish_integration import fake_bridge, fake_state, imu_delivery, bar_delivery, Stamp
        from bridge.ros2_publish_runtime import flush_sensor_packet_jobs
        from bridge.ros2_publisher_demand import PublisherDemandCache
        owner = fake_bridge()
        owner._real_pkg_compat = True
        owner._mavros_surface_enabled = False
        owner._strict_sitl_sensor_transport = True
        owner.pub_mavros_imu_data_raw = 'raw'
        owner.pub_mavros_imu_static_pressure = 'pressure'
        owner._publisher_demand = PublisherDemandCache()
        delivered = []
        owner._safe_publish = lambda publisher, msg, label: delivered.append((publisher, msg)) or True
        state = fake_state((imu_delivery(1., 7.),), (bar_delivery(1., 110000.),))
        state.sim_t = 1.05
        self.assertTrue(flush_sensor_packet_jobs(owner, Stamp(), state))
        self.assertEqual([publisher for publisher, _ in delivered], ['raw', 'pressure'])
        self.assertEqual(delivered[0][1].header.stamp.sec, 1)
        self.assertEqual(delivered[0][1].angular_velocity.x, 7.)
        self.assertEqual(delivered[0][1].orientation_covariance[0], -1.)

    def test_sitl_step_feeds_fcu_when_ros_not_due(self):
        runtime = SimpleNamespace(sitl_enabled=True, publish_ros_once=Mock(), publish_qgc_video_once=Mock())
        with patch('sim.runtime.simulation_step_raw_pwm._apply_raw_pwm_control_inputs', return_value=(0,0,0,0)), \
             patch('sim.runtime.simulation_step_raw_pwm.apply_common_step_physics'), \
             patch('sim.runtime.simulation_step_raw_pwm.record_step_phase'):
            run_raw_pwm_runtime_step(runtime, is_paused=False, publish_ros=False, thruster_due=False, thruster_dt=.0025)
        runtime.publish_ros_once.assert_called_once()

    def test_fcu_measurement_rate_is_not_host_packet_rate(self):
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            self.assertEqual(bridge._imu_sensor_timing.config.schedule.rate_hz, 400.)
            packets = []
            samples = []
            for i in range(41):
                t = i * .0025
                measured, _, deliveries, _ = advance_imu_bar30_sensor_runtime(
                    bridge, base_state(t), replace(imu_state(), gyro_bmj=imu_state().gyro_bmj * (i+1)), vertical_state())
                samples.append(float(measured.gyro_bmj[0]))
                packets.extend(deliveries)
            self.assertEqual(len(set(samples)), 41)
            self.assertEqual(len(packets), 6, 'host export remains 50 Hz')


if __name__ == '__main__':
    unittest.main()
