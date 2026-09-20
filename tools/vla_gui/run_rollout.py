"""Bounded simulation-only policy trial; truth is evaluation-only, never policy input."""
import argparse,json,time,math
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn,State
from rosgraph_msgs.msg import Clock

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--dry_run',action='store_true');p.add_argument('--seconds',type=float,default=30);p.add_argument('--task',default='Approach the yellow buoy, align the fixed fork, and detach the buoy.');a=p.parse_args()
rclpy.init();node=rclpy.create_node('vla_bounded_sim_trial')
state={'clock':0.,'poses':[],'rc':[],'buoys':[],'vehicle':{}}
dead=node.create_publisher(Bool,'/vla/deadman',10);task=node.create_publisher(String,'/vla/task_description',10)
client=node.create_client(SetBool,'/vla/enable')
def odom(m):
 t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;p=m.pose.pose.position;v=m.twist.twist.linear
 state['poses'].append([t,p.x,p.y,p.z,v.x,v.y,v.z])
def buoy(m):
 try:state['buoys'].append(json.loads(m.data))
 except ValueError:pass
node.create_subscription(Odometry,'/sim/odom',odom,qos_profile_sensor_data)
node.create_subscription(Clock,'/clock',lambda m:state.update(clock=m.clock.sec+m.clock.nanosec*1e-9),10)
node.create_subscription(String,'/mujoco/course_buoys/status',buoy,10)
node.create_subscription(State,'/mavros/state',lambda m:state.update(vehicle=dict(connected=m.connected,armed=m.armed,mode=m.mode)),qos_profile_sensor_data)
node.create_subscription(OverrideRCIn,'/vla/proposed_rc' if a.dry_run else '/mavros/rc/override',lambda m:state['rc'].append([state['clock'],[int(v) for v in m.channels]]),10)
def beat():
 dead.publish(Bool(data=True));task.publish(String(data=a.task))
timer=node.create_timer(.08,beat)
def call(value,timeout=5):
 req=SetBool.Request();req.data=value;future=client.call_async(req);end=time.monotonic()+timeout
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 if not future.done():raise RuntimeError('enable service timeout')
 return future.result()
result={'dry_run':a.dry_run,'task':a.task,'outcome':'not_started'}
try:
 end=time.monotonic()+15
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.02)
  if client.service_is_ready() and state['clock']>0 and state['vehicle']:
   enabled=call(True)
   if enabled.success:
    result.pop('enable_message',None);break
   result['enable_message']=enabled.message
 else:raise RuntimeError('Policy enable failed: '+result.get('enable_message','missing clock/service'))
 start=state['clock'];wall=time.monotonic();state['poses']=[];state['rc']=[];state['buoys']=[]
 result.update(outcome='time_limit',start_sim_time=start)
 while state['clock']-start<a.seconds and time.monotonic()-wall<120:
  rclpy.spin_once(node,timeout_sec=.02)
  if state['buoys'] and any(b.get('detached') for b in state['buoys'][-1].get('buoys',[])):
   result['outcome']='detached';break
  if state['poses'] and max(abs(x) for x in state['poses'][-1][4:])>3:
   result['outcome']='velocity_guard';break
  if not a.dry_run and not state['vehicle'].get('armed'):
   result['outcome']='disarmed';break
  if time.monotonic()-wall>8 and len(state['rc'])<3:
   result['outcome']='no_policy_output';break
 result['sim_duration_s']=state['clock']-start
 result['rc_frames']=len(state['rc'])
 result['nonneutral_frames']=sum(any(v not in (0,65535) and abs(v-1500)>5 for v in ch[:8]) for _,ch in state['rc'])
 if state['poses']:
  first,last=state['poses'][0],state['poses'][-1];result['xy_displacement_m']=math.hypot(last[1]-first[1],last[2]-first[2])
except Exception as e:result.update(outcome='error',error=str(e))
finally:
 timer.cancel();dead.publish(Bool(data=False))
 if client.service_is_ready():
  try:call(False)
  except Exception as e:result['disable_error']=str(e)
 result['telemetry']=state;a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='telemetry'},indent=2),flush=True)
 node.destroy_node();rclpy.shutdown()
