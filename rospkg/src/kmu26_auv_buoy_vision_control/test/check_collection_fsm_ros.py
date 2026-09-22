"""ROS contract test in a separate ROS_DOMAIN_ID; no simulator or vehicle required."""
import os
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32MultiArray, String
from mavros_msgs.msg import OverrideRCIn

assert os.environ.get('ROS_DOMAIN_ID') == '155', 'Use isolated test ROS_DOMAIN_ID=155'
rclpy.init()
node = rclpy.create_node('collection_contract_test')
clock = node.create_publisher(Clock, '/clock', 10)
depth = node.create_publisher(PoseWithCovarianceStamped, '/depth/pose', 10)
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
request = node.create_publisher(Bool, '/auto_collection/search', qos)
grant = node.create_publisher(Bool, '/auto_collection/grant', qos)
front = node.create_publisher(Float32MultiArray, '/auto_collection/bbox', 10)
hand = node.create_publisher(Float32MultiArray, '/auto_collection/hand_bbox', 10)
received = []
phases = []
node.create_subscription(OverrideRCIn, '/auto_collection/proposed_rc', lambda m: received.append(list(m.channels)), 10)
node.create_subscription(String, '/mission/state', lambda m: phases.append(m.data), 10)
executable = os.environ.get('FSM_TEST_EXECUTABLE')
from ament_index_python.packages import get_package_prefix
from pathlib import Path
command = [executable or str(Path(get_package_prefix('auv_buoy_vision_control')) / 'lib/auv_buoy_vision_control/collection_fsm_node')]
target_x, target_y = .45, .60
if os.environ.get('FSM_TEST_HAND_CALIBRATION'):
    import json
    profile = json.loads(Path(os.environ['FSM_TEST_HAND_CALIBRATION']).read_text())
    target_x, target_y = profile['hand_target_uv']
    inv = profile['error_to_right_down_m']
    values = dict(hand_target_x=target_x, hand_target_y=target_y,
                  hand_right_error_x=inv[0][0]/.1, hand_right_error_y=inv[0][1]/.1,
                  hand_depth_error_x=inv[1][0]/.7, hand_depth_error_y=inv[1][1]/.7)
    command += ['--ros-args'] + [item for name,value in values.items()
                                for item in ['-p',f'{name}:={value}']]
process = subprocess.Popen(command + ['--ros-args', '-p', 'use_sim_time:=true',
    '-p', 'bbox_topic:=/auto_collection/bbox', '-p', 'rc_override_topic:=/auto_collection/proposed_rc',
    '-p', 'vision_search_request_topic:=/auto_collection/search',
    '-p', 'vision_control_granted_topic:=/auto_collection/grant'], stdout=subprocess.DEVNULL)
t = 10.
def tick(n, front_visible=False, hand_x=None, hand_y=None, front_height=60.):
    if hand_y is None: hand_y = target_y
    global t
    for _ in range(n):
        t += .1
        stamp=Clock();stamp.clock.sec=int(t);stamp.clock.nanosec=int((t-int(t))*1e9);clock.publish(stamp)
        pose=PoseWithCovarianceStamped();pose.pose.pose.position.z=-1.;depth.publish(pose)
        if front_visible:
            front.publish(Float32MultiArray(data=[t,1.,0.,.9,320.,180.,30.,front_height,640.,360.]))
        if hand_x is not None:
            hand.publish(Float32MultiArray(data=[t,1.,1.,.9,hand_x*640.,hand_y*360.,20.,110.,640.,360.]))
        end=time.monotonic()+.06
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
try:
    tick(25)
    request.publish(Bool(data=True));grant.publish(Bool(data=True))
    tick(70)
    assert len(set(row[2] for row in received)) > 5, 'SEARCH depth target stayed fixed'
    received.clear()
    tick(15, True, front_height=100.)
    assert received and all(row[4] == 1500 for row in received[-5:]), 'Hand acquisition did not stop advance'
    assert any(row[3] < 1500 for row in received[-5:]), 'No rotation toward hand field of view'
    received.clear()
    t += 2.0  # Front observation expires before the first hand observation.
    tick(30,False,.8)
    assert 'ALIGN_STICK' in phases, phases
    assert any(row[5]>1500 and row[4]==1500 for row in received), 'No hand-based lateral alignment / forward gating'
    received.clear()
    tick(10,True,.8,.1)
    def effective(pwm):
        delta=int(pwm)-1500
        return max(0,abs(delta)-30)*(1 if delta>0 else -1)
    assert received and max(abs(effective(b[2])-effective(a[2])) for a,b in zip(received,received[1:])) < 30, 'Visual target caused a depth derivative kick'
    received.clear()
    tick(30,True,.1)
    assert any(row[5]<1500 for row in received), 'No opposite lateral correction'
    received.clear()
    tick(12, True, target_x + .08)
    assert all(row[4] == 1500 for row in received[-5:]), 'Advance continued before fine hand alignment'
    tick(25,True,target_x)
    assert 'INSERT_FORK' in phases, phases
    print('PASS search depth sweep, hand handoff, bidirectional sway, forward gate, insertion transition')
finally:
    process.terminate();process.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()
