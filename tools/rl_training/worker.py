"""One isolated ROS/SITL rollout; the parent supplies VLA+residual actions."""
import json,sys,time,threading,math,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from kmu26_auv_vla_data_collector.collector import VlaDataCollector
from reward import ForkReward,RewardConfig
from precision_reward import PrecisionReward
from ipc import write,read,status
from recording import finish_recording
from recovery import physics_failure
import urllib.request
import urllib.error

folder=Path(sys.argv[1]);config=json.loads((folder/'config.json').read_text());sys.argv=[sys.argv[0]]
stop=threading.Event();signal.signal(signal.SIGTERM,lambda *_:stop.set())
def api(command=None,**kw):
 data=None if command is None else json.dumps(dict(command=command,**kw)).encode()
 url='http://127.0.0.1:8878'+('/api/status' if data is None else '/api/command')
 try:
  with urllib.request.urlopen(urllib.request.Request(url,data=data,headers={'Content-Type':'application/json'}),timeout=5) as r:
   result=json.load(r)
 except urllib.error.HTTPError as error:
  raise RuntimeError(str(command)+': '+error.read().decode()) from error
 if result.get('error'):raise RuntimeError(result['error'])
 return result

def wait(fn,label,timeout=90):
 end=time.monotonic()+timeout
 while time.monotonic()<end and not stop.is_set():
  if globals().get('owned',False):control(previous)
  result=fn()
  if result:return result
  time.sleep(.05)
 raise RuntimeError('Waiting failed: '+label)

import yaml
parameters=yaml.safe_load(Path('/workspace/rospkg/src/kmu26_auv_vla_policy/config/sim_policy.yaml').read_text())['vla_data_collector']
parameters['ros__parameters']['default_task']='Approach the yellow buoy, align the fixed fork, and detach the buoy.'
parameters['ros__parameters']['dataset_root']=str(folder/'sensor-staging')
(folder/'sensors.yaml').write_text(yaml.safe_dump({'rl_sensors':parameters}))
rclpy.init(args=['--ros-args','--params-file',str(folder/'sensors.yaml'),'-r','__node:=rl_sensors'])
sensors=VlaDataCollector();executor=SingleThreadedExecutor();executor.add_node(sensors)
truth={};truth_wall=0.
def on_truth(message):
 global truth,truth_wall
 truth=json.loads(message.data);truth_wall=time.monotonic()
sensors.create_subscription(String,'/mujoco/course_buoys/status',on_truth,10)
thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
previous=np.zeros(4);sequence=0;owned=False;recording=False;completed=False
client='rl-'+folder.name
mapping={'general_release':'general_release_reward','hand_release':'hand_release_reward','approach_progress':'approach_progress_reward','contact_bonus':'contact_bonus_reward','deadline_s':'episode_seconds','success':'success_reward','fast_success':'fast_success_reward','timeout':'timeout_reward','wrong_release':'wrong_release_reward','height_progress':'height_progress_reward','rediscovery':'rediscovery_reward','time_cost_per_s':'time_cost_per_second'}
rcfg=RewardConfig(**{k:config.get(mapping.get(k,k), getattr(RewardConfig(),k)) for k in RewardConfig.__dataclass_fields__})
reward=(PrecisionReward if config.get('reward_version') == 'precision_v3' else ForkReward)(rcfg)
def sensor_observation():
 try:return sensors.policy_observation(previous)
 except ValueError:return None

def control(action):
 global sequence,previous,owned
 previous=np.asarray(action,dtype=float);sequence+=1
 response=api('rc',client_id=client,seq=sequence,enabled=True,
              axes=dict(zip(('forward','lateral','heave','yaw'),previous.tolist())))
 if response.get('accepted') is False:raise RuntimeError('RC ownership rejected')
 owned=True

def measurement(target,obs):
 if time.monotonic()-truth_wall>3:raise RuntimeError('Stale simulation reward telemetry')
 row=next(b for b in truth['buoys'] if b['id']==target)
 q=np.asarray(obs['state.attitude']).reshape(-1);w,x,y,z=q
 result = dict(time_s=truth['time_s'],target_id=target,detached=row['detached'],eq_active=row['eq_active'],
  release_right_fork_region=row.get('release_right_fork_region',False),
  release_time_s=row['release_time_s'],release_right_fork_stem_contact=row['release_right_fork_stem_contact'],
  height_error_m=row['rl_height_error_m'],fork_stem_distance_m=row['rl_fork_stem_distance_m'],
  right_fork_stem_contact_active=row.get('rl_right_fork_stem_contact_active',False),
  hand_visible=row.get('rl_hand_target_line_of_sight',False),
  camera_valid=True,visible=row['rl_target_line_of_sight'],depth_m=float(np.asarray(obs['state.depth']).reshape(-1)[0]),
  yaw_rad=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if config.get('reward_version') == 'precision_v3':
  result['slot_offset_m'] = row['rl_slot_offset_m']
  result['yaw_rate_rad_s'] = float(np.asarray(obs['state.angular_velocity']).reshape(-1)[2])
  if not np.isfinite(result['slot_offset_m']).all() or not math.isfinite(result['yaw_rate_rad_s']):
   raise RuntimeError('Invalid precision telemetry')
 return result

try:
 def fcu_ready():
  status=api();telemetry=status['telemetry']
  return (status['processes'].get('mavros_running')
          and 'auto->mavros' in status['backend']['label']
          and telemetry.get('connected')
          and telemetry.get('state_age_s') is not None
          and telemetry['state_age_s'] < 1.)
 wait(fcu_ready,'FCU connected',120)
 mode_attempts=0;mode_last_request=0.
 def mode_ready():
  global mode_attempts,mode_last_request
  if api()['telemetry'].get('mode')=='STABILIZE':return True
  now=time.monotonic()
  if mode_attempts < 5 and now-mode_last_request >= 5. and fcu_ready():
   api('mode',mode='STABILIZE');mode_attempts+=1;mode_last_request=now
  return False
 wait(mode_ready,'mode')
 if not api()['telemetry'].get('armed'):api('arm',value=True)
 wait(lambda:api()['telemetry']['armed'],'arm')
 control(np.zeros(4))
 if not api()['recorder'].get('running'):
  api('recorder_prepare',task='Approach the yellow buoy, align the fixed fork, and detach the buoy.',mode='STABILIZE')
 wait(lambda:api()['recorder'].get('ready'),'recorder')
 api('release',client_id=client);owned=False
 api('recorder_action',action='reset')
 wait(lambda: not api()['recorder'].get('busy'),'reset')
 if not api()['recorder'].get('message','').startswith('완료:'):
  raise RuntimeError('Map reset did not complete successfully')
 control(np.zeros(4))
 obs=wait(sensor_observation,'fresh observations')
 wait(lambda: truth.get('buoys'),'buoy truth')
 targets=[b for b in truth['buoys'] if b.get('eq_active') and b.get('color')=='yellow']
 if not targets:raise RuntimeError('No attached yellow target')
 target=min(targets,key=lambda b:b['rl_fork_stem_distance_m'])['id']
 # Warm inference before starting the episode clock or recording.
 write(folder/'request.pkl',dict(id=-1,observation=obs,warmup=True))
 wait(lambda:(folder/'response.pkl').exists(),'model warmup',120)
 (folder/'response.pkl').unlink()
 wait(lambda:api()['recorder'].get('ready'),'recorder after reset and warmup',30)
 previous_recording_path=api()['recorder'].get('last_result',{}).get('path')
 api('recorder_action',action='start');wait(lambda:api()['recorder'].get('active'),'recording');recording=True
 obs=wait(sensor_observation,'initial observation')
 total=0.;trace=[];index=0;feedback=None;wall_start=time.monotonic()
 last_applied_id=-1;last_time=None;action_started=None;observation_time=None
 while not stop.is_set():
  vehicle=api()['telemetry']
  if not vehicle.get('armed') or vehicle.get('mode')!='STABILIZE':raise RuntimeError('Vehicle arm/mode changed')
  obs=wait(sensor_observation,'observation',5)
  request_time=float(truth['time_s'])
  write(folder/'request.pkl',dict(id=index,observation=obs,feedback=feedback,warmup=False))
  feedback=None
  deadline=time.monotonic()+10
  while not (folder/'response.pkl').exists():
   if stop.is_set() or time.monotonic()>deadline:raise RuntimeError('Policy response timeout')
   control(previous);time.sleep(.05)
  response=read(folder/'response.pkl');(folder/'response.pkl').unlink()
  if response['id']!=index:raise RuntimeError('Policy response sequence mismatch')
  if response.get('error'):raise RuntimeError(response['error'])
  # Close the PREVIOUS applied command before installing the pending command.
  # Inference latency belongs to the command held during that latency.
  obs_now=wait(sensor_observation,'command boundary observation',5)
  measure=measurement(target,obs_now)
  if last_applied_id >= 0:
   result=reward.step(measure)
   feedback=dict(**result,action_id=last_applied_id,duration=measure['time_s']-last_time,
                 time_s=measure['time_s'],action_started_s=action_started,
                 observation_time_s=observation_time,command_boundary_s=measure['time_s'])
   total+=result['reward'];trace.append(dict(**feedback,measurement=measure,action=previous.tolist()))
   status(folder/'progress.json',dict(sim_seconds=measure['time_s']-reward.start,reward=total,steps=len(trace)))
   if result['done']:break  # Pending command is never applied or trained.
  else:
   reward.reset(measure)
  last_time=measure['time_s'];action_started=last_time;observation_time=request_time
  control(response['action']);last_applied_id=index
  wait(lambda:truth.get('time_s',0)>=action_started+.1,'simulation tick',10)
  index+=1
  if time.monotonic()-wall_start>max(300,config['episode_seconds']*30):raise RuntimeError('Wall-time watchdog')
 if stop.is_set():raise RuntimeError('Stopped')
 control(np.zeros(4))
 saved,recording_complete=finish_recording(api,wait,previous_path=previous_recording_path,success=result['released'])
 recording=False
 if not recording_complete:
  status(folder/'recording_warning.json',dict(reason=saved['termination_reason'],recording=saved,usable_as_demonstration=False))
 write(folder/'request.pkl',dict(id=index,done=True,last_applied_id=last_applied_id,feedback=feedback,total_reward=total,success=result['success'],released=result['released'],release_tier=result['release_tier'],precision_best=result.get('precision_best')))
 status(folder/'episode.json',dict(success=result['success'],released=result['released'],release_tier=result['release_tier'],reward=total,trace=trace,recorder=saved,recording_complete=recording_complete,precision_best=result.get('precision_best')))
 completed=True
except Exception as error:
 try:status(folder/'failure_status.json',api())
 except Exception:pass
 fault=physics_failure('/workspace/uuv_mujoco/current/logs')
 status(folder/'error.json',dict(error=(fault['detail'] if fault else str(error)),original_error=str(error),**(fault or {})))
 status(folder/'failed-trajectory.json',dict(usable_for_training=False,trace=globals().get('trace',[]),last_applied_id=globals().get('last_applied_id',-1),reason=str(error)))
 raise
finally:
 try:
  if owned:api('release',client_id=client)
  if recording and api()['recorder'].get('active'):api('recorder_action',action='failure')
  if not completed or config.get('_final_wave',True):api('arm',value=False)
 except Exception:pass
 executor.shutdown(timeout_sec=10)
 thread.join(timeout=10)
 sensors.destroy_node();rclpy.shutdown()
